#!/usr/bin/perl

=head1 navilink.pl

This Script implements the NaviLink protocol to communicate with a
Locosys NaviGPS via serial USB

Please note: This script is far from complete, only track downloading is
             currently supported. If you want to help refer to the protocol
             specification at http://wiki.splitbrain.org/navilink at send
             patches in unified diff format.


Copyright (c) 2007, Andreas Gohr <andi (at) splitbrain.org>

Contributors:
    Andreas Gohr <andi (at) splitbrain.org>
    Martijn van Oosterhout <kleptog (at) svana.org>

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

    * Neither the name of Andreas Gohr nor the names of other contributors may
      be used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=cut


$| = 1;

use Device::SerialPort;
use Data::Dumper;
use Getopt::Std;

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

my %OPT;

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

    print STDERR '-> '.hexdump($packet)."\n" if($OPT{'v'});
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
        
        while( length($msg) > 2 and substr($msg,0,2) ne "\xA0\xA2" )
        {
          $msg = substr($msg,1);
        }

        if( (time() - $start) > 8){
            printf STDERR "Timeout while reading. Last bytes %s\n", hexdump(substr($msg,-10));
            last;
        }
    } until (length($msg) > 4 and length($msg)-8 == unpack("v", substr($msg,2,2)));

    print STDERR '<- '.hexdump($msg)."\n" if($OPT{'v'});

    my $payload = substr($msg,4,-4);
    my $sum     = unpack("v",substr($msg,-4,2));
    if($sum != checkSum($payload))
    {
        print STDERR "Expected checksum ",$sum," got ",checkSum($payload),"\n";
        print STDERR "Packet: ",hexdump($msg),"\n";
        die "aarrgh!\n";
    }
    my $type = substr($payload,0,1);
    my $data = substr($payload,1);

    return ($type,$data);
}

=head2 downloadInfo

Reads general information from the device and returns it as hash

=cut
sub downloadInfo {
    sendRawPacket(PID_QRY_INFORMATION,"");
    my ($type,$data) = readPacket();
    if($type ne PID_DATA){
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
    if(!defined(%info) || !$info{'trackpoints'}){
        print STDERR "There are no trackpoints available";
        return 0;
    }
    my ($type,$data);
    my $addr  = $info{'trackbuffer'};        # buffer address
    my $read  = 0;                           # bytes already read
    my $max   = $info{'trackpoints'} * 32;   # maximum bytes to read
    my $track = '';

    while($read < $max){
        my $toread = 512*32;
        $toread = ($max - $read) if( ($max - $read) < $toread);

        # request data
        my $msg = pack("V",$addr+$read).pack("v",$toread)."\x00";
        sendRawPacket(PID_READ_TRACKPOINTS,$msg);


        # read answer
        ($type,$data) = readPacket();
        if($type ne PID_DATA){
            # something went wrong
            print STDERR "Did not receive expected Trackpoint data";
            return 0;
        }
        $track .= $data;

        # increase read counter
        $read += $toread;

        # send ACK
        sendRawPacket(PID_ACK,"");
    }

    # Creat GPX file
    print OUT "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    print OUT "<gpx version=\"1.0\" creator=\"navilink\"
                xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"
                xmlns=\"http://www.topografix.com/GPX/1/0\"
                xsi:schemaLocation=\"http://www.topografix.com/GPX/1/0 http://www.topografix.com/GPX/1/0/gpx.xsd\">\n";
    print OUT "<trk>\n";
    print OUT "<trkseg>\n";

    # now parse the track log
    for(my $i=0; $i<$read; $i+=32){
        my @tp = unpack("vvllllvCCCCCCCCCC",substr($track,$i,32));

        #print hexdump(substr($track,$i,32))."\n";
        #print Dumper(\@tp);

        printf OUT "<trkpt lat=\"%f\" lon=\"%f\">\n", $tp[4]/10000000, $tp[5]/10000000;
        printf OUT "  <ele>%f</ele>\n", $tp[6]*0.3048; #feet to meters
        printf OUT "  <time>%04d-%02d-%02dT%02d:%02d:%02d</time>\n", $tp[7]+2000, $tp[8], $tp[9], $tp[10], $tp[11], $tp[12];
        printf OUT "  <speed>%02d</speed>\n",$tp[14]*2;
        printf OUT "</trkpt>\n";
    }

    print OUT "</trkseg>\n";
    print OUT "</trk>\n";
    print OUT "</gpx>\n";

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

sub help {
    print <<EOT;
Usage: navilink.pl [OPTIONS] COMMAND
Download or Upload data to a NaviGPS device

  -h            print this usage help
  -v            be verbose (packet debugging)
  -d <device>   device to use, defaults to /dev/ttyUSB0
  -q            quit communication after finishing
  -i <file>     use this file for input instead of STDIN
  -o <file>     use this file for output instead of STDOUT

COMMAND can be one of these:

  info          print number of waypoints, routes and trackpoints
  gettracks     download track data as GPX
EOT

    exit 0
}

###############################################################################
# main


# prepare options
getopts('d:hvqi:o:',\%OPT);
$OPT{'d'} = '/dev/ttyUSB0' if(!$OPT{'d'});
help() if($OPT{'h'} || !$ARGV[0]);


# open device
$DEV = new Device::SerialPort ($OPT{'d'}) || die "Can't open ".$OPT{'d'}.": $^E\n";
$DEV->baudrate(115200);
$DEV->databits(8);
$DEV->parity("none");
$DEV->stopbits(1);

# sync
sendRawPacket(PID_SYNC,"");
my ($type,$data) = readPacket();
die("got no ack on sync") if($type ne PID_ACK);

# open files
if($OPT{'i'}){
    open(IN,$OPT{'i'}) || die("Could not open ".$OPT{'i'}." for reading");
}else{
    *IN = *STDIN;
}

if($OPT{'o'}){
    open(OUT,">".$OPT{'o'}) || die("Could not open ".$OPT{'o'}." for writing");
}else{
    *OUT = *STDERR;
}



# handle commands
if($ARGV[0] eq 'gettracks'){
    downloadTrackData();
}elsif($ARGV[0] eq 'info'){
    my %info = downloadInfo();
    print OUT 'waypoints   : '.$info{'waypoints'}."\n";
    print OUT 'routes      : '.$info{'routes'}."\n";
    print OUT 'trackpoints : '.$info{'trackpoints'}."\n";
}


# quit if wanted
sendRawPacket(PID_QUIT,"") if ($OPT{'q'});

$DEV->close;
close(IN);
close(OUT);
exit 0;

