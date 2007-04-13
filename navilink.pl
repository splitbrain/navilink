#!/usr/bin/perl



$device = '/dev/ttyUSB0';

use Device::SerialPort;

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

Sends a package to be send over the line. Data needs to be in correct
endianess already.

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

    # debugging -> both should output the same
    #print "A0A20100D6D600B0B3\n";
    print "-> ".unpack("H*",$packet)."\n";
    $DEV->write($packet);
}

sub readPacket {
    my ($rl, $b, $msg);

    $msg = "";
    do {
        ($rl, $b) = $DEV->read(1);
        $msg .= $b;
    } until ($msg =~ /(\xA0\xA2.+?\xB0\xB3)/);

    print "<- ".unpack("H*",$msg)."\n";

    my $payload = substr($msg,4,-4);
    my $sum     = unpack("v",substr($msg,-4,2));
    die "Checksum failed" if($sum != checkSum($payload));
    my $type = substr($payload,0,1);
    my $data = substr($payload,1);

    return ($type,$data);
}

sub parseInfo {
    my $data = shift;
    my @info = unpack('vCCVVvv',$data);

    printf "Waypoints  : %d\n", $info[0];
    printf "Routes     : %d\n", $info[1];
    printf "Tracks     : %d\n", $info[2];
    printf "AddrBuffer : %d\n", $info[3];
    printf "Serial     : %d\n", $info[4];
    printf "Trackpoints: %d\n", $info[5];
    printf "Protocol   : %d\n", $info[6];
    printf "Username   : %s\n", substr($data,-16);
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


$DEV = new Device::SerialPort ($device) || die "Can't open $device: $^E\n";
$DEV->baudrate(115200);
$DEV->databits(8);
$DEV->parity("none");
$DEV->stopbits(1);

=pod
print $DEV->write("\xA0\xA2\x01\x00\xD6\xD6\x00\xB0\xB3");

($count_in, $string_in) = $DEV->read(5);
print "$count_in $string_in";
=cut

#print PID_SYNC;

sendRawPacket(PID_SYNC);
($type,$data) = readPacket();
die("got no ack on sync") if($type != PID_ACK);

sendRawPacket(PID_QRY_INFORMATION);
($type,$data) = readPacket();
parseInfo($data);

sendRawPacket(PID_QUIT);


$DEV->close;


