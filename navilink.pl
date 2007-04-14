#!/usr/bin/perl

$| = 1;

use Device::SerialPort;
use Time::HiRes qw( usleep );
use Data::Dumper;

use constant PID_SYNC             => "\xd6";
use constant PID_ACK              => "\x0c";
use constant PID_NAK              => "\x00";
use constant PID_QRY_INFORMATION  => "\x20";
use constant PID_DATA             => "\x03";
use constant PID_ADD_A_WAYPOINT   => "\x3C";
use constant PID_QRY_WAYPOINTS    => "\x28";
use constant PID_QRY_ROUTE        => "\x24";
use constant PID_DEL_WAYPOINT     => "\x36";
use constant PID_DEL_ALL_WAYPOINT => "\x37";
use constant PID_DEL_ROUTE        => "\x34";
use constant PID_DEL_ALL_ROUTE    => "\x35";
use constant PID_ADD_A_ROUTE      => "\x3D";
use constant PID_ERASE_TRACK      => "\x11";
use constant PID_READ_TRACKPOINTS => "\x14";
use constant PID_CMD_OK           => "\xf3";
use constant PID_CMD_FAIL         => "\xf4";
use constant PID_QUIT             => "\xf2";


=head2 sendRawPacket I<type> I<data>

Sends a package over the line.

=cut
sub sendRawPacket {
    my $type = shift();
    my $data = shift();

    my $packet = "\xA0\xA2";                         # start sequence
       $packet .= pack('v',length($data)+1);         # payload length (incl. packet type)
       $packet .= $type;                             # packet type
       $packet .= $data;                             # data
       $packet .= pack('v',checkSum($type.$data));   # checksum
       $packet .= "\xB0\xB3";                        # end sequence

    print STDERR '-> '.hexdump($packet)."\n";
    $DEV->write($packet);
}

=head2 readPacket

Read a packet from the line and return type and data in an array.

=cut
sub readPacket {
    my ($rl, $b, $msg);

    my $start = time();

    $msg = "";
    do {
        ($rl, $b) = $DEV->read(1);
        $msg .= $b;

        if( (time() - $start) > 8){
            printf STDERR "Timeout while reading. Last bytes %s\n", hexdump(substr($msg,-10));
            last;
        }
    } until (substr($msg,-2) eq "\xb0\xb3");

    print STDERR '<- '.hexdump($msg)."\n";

    my $payload = substr($msg,4,-4);
    my $sum     = unpack("v",substr($msg,-4,2));
    die "Checksum failed" if($sum != checkSum($payload));
    my $type = substr($payload,0,1);
    my $data = substr($payload,1);

    return ($type,$data);
}

=head2 downloadInfo

Reads general information from the device and returns it as hash

=cut
sub downloadInfo {
    sendRawPacket(PID_QRY_INFORMATION);
    my ($type,$data) = readPacket();
    if($type != PID_DATA){
        print STDERR "got no info data\n";
        return undef;
    }

    my @bytes = unpack('vCCVVvv',$data);

    my %info = (
        'waypoints'   => $bytes[0],
        'routes'      => $bytes[1],
        'tracks'      => $bytes[2],
        'trackbuffer' => $bytes[3],
        'serial'      => $bytes[4],
        'trackpoints' => $bytes[5],
        'protocol'    => $bytes[6],
        'username'    => substr($data,-16)
    );

    return %info;
}

=head2 downloadTrackData

Reads all Trackpoints from the device and prints it as GPX data

=cut
sub downloadTrackData {
    my %info = downloadInfo();

    print STDERR Dumper(\%info);

    if(!defined(%info) || !$info{'trackpoints'}){
        print STDERR "There are no trackpoints available";
        return 0;
    }
    my $type,$data;
    my $addr  = $info{'trackbuffer'};        # buffer address
    my $read  = 0;                           # bytes already read
    my $max   = $info{'trackpoints'} * 32;   # maximum bytes to read
#$max = 64; #FIXME two wp only
    my $track = '';

    while($read < $max){
        my $toread = 512*32;
        $toread = ($max - $read) if( ($max - $read) < $toread);

        # request data
        my $msg = pack("V",$addr+$read).pack("v",$toread)."\x00";
        sendRawPacket(PID_READ_TRACKPOINTS,$msg);


        # read answer
        ($type,$data) = readPacket();
        if($type != PID_DATA){
            # something went wrong
            print STDERR "Did not receive expected Trackpoint data";
            return 0;
        }

        $track .= $data;

        # FIXME handle date here

        # increase read counter
        $read += $toread;

        # send ACK
        sendRawPacket(PID_ACK);
    }


    print "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    print "<gpx version=\"1.0\" creator=\"navilink\"
            xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"
            xmlns=\"http://www.topografix.com/GPX/1/0\"
            xsi:schemaLocation=\"http://www.topografix.com/GPX/1/0 http://www.topografix.com/GPX/1/0/gpx.xsd\">\n";
    print "<trk>\n";
    print "<trkseg>\n";

    # now parse the track log
    for(my $i=0; $i<$read; $i+=32){
        my @tp = unpack("vvllllvCCCCCCCCCC",substr($track,$i,32));

        #print hexdump(substr($track,$i,32))."\n";
        #print Dumper(\@tp);

        printf "<trkpt lat=\"%f\" lon=\"%f\">\n", $tp[4]/10000000, $tp[5]/10000000;
        printf "  <ele>%f</ele>\n", $tp[6]*0.3048; #feet to meters
        printf "  <time>%04d-%02d-%02dT%02d:%02d:%02d</time>\n", $tp[7]+2000, $tp[8], $tp[9], $tp[10], $tp[11], $tp[12];
        printf "  <speed>%02d</speed>\n",$tp[14]*2;
        printf "</trkpt>\n";
    }

    print "</trkseg>\n";
    print "</trk>\n";
    print "</gpx>\n";

    return 1;
}

=head2 checkSum I<payload>

Calculates a checksum for the given payload

=cut
sub checkSum {
    my @payload = unpack('C*',shift());
    my $sum = 0;

    foreach my $c (@payload){
        $sum += $c;
    }
    $sum = $sum & 0x7fff;
    return $sum;
}

=head2 hexdump I<packet>

Formats the given data as hex

=cut
sub hexdump {
    my $packet = shift;

    if(length($packet) > 60){
        return hexdump(substr($packet,0,6)).'['.
               length(substr($packet,6,-6)).' more bytes] '.
               hexdump(substr($packet,-6));

    }


    $packet = unpack("H*", $packet);
    $packet =~ s/(\S{2})/$1 /g;
    $packet =~ s/(\S{2} \S{2} \S{2} \S{2})/$1 /g;

    return $packet;
}


###############################################################################
# main

$device = '/dev/ttyUSB0'; #FIXME make configurable

$DEV = new Device::SerialPort ($device) || die "Can't open $device: $^E\n";
$DEV->baudrate(115200);
$DEV->databits(8);
$DEV->parity("none");
$DEV->stopbits(1);

sendRawPacket(PID_SYNC);
($type,$data) = readPacket();
die("got no ack on sync") if($type != PID_ACK);

#sendRawPacket(PID_QRY_INFORMATION);
#($type,$data) = readPacket();
#die("got no data") if($type != PID_DATA);
#%info = parseInfoPacket($data);


downloadTrackData();

#sendRawPacket(PID_QUIT);


$DEV->close;


