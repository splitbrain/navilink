#!/usr/bin/perl

=head1 navilink.pl

This Script implements the NaviLink protocol to communicate with a
Locosys NaviGPS via serial USB

Please note: This script is far from complete. If you want to help,
             refer to the protocol specification at
             http://wiki.splitbrain.org/navilink at send patches
             in unified diff format.


Copyright (c) 2007-2009, Andreas Gohr <andi (at) splitbrain.org>

Contributors:
    Andreas Gohr <andi (at) splitbrain.org>
    Martijn van Oosterhout <kleptog (at) svana.org>
    Nick Lamb
    Richard Fairhurst <richard (at) systemeD.net>
    Tim Aerts <tim__aerts@hotmail.com>

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
use constant PID_QRY_FW_VERSION   => "\xfe";
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
use constant PID_READ_LOG_HEADER  => "\x50";
use constant PID_LOG_DATA_ADDR    => "\x1c";
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
            printf STDERR "Timeout while reading. Last bytes where '%s'.\n",
                   hexdump(substr($msg,-10));
            return undef;
        }
    } until (length($msg) > 4 and length($msg)-8 == unpack("v", substr($msg,2,2)));

    print STDERR '<- '.hexdump($msg)."\n" if($OPT{'v'});

    my $payload = substr($msg,4,-4);
    my $sum     = unpack("v",substr($msg,-4,2));
    if($sum != checkSum($payload)) {
        nicedie('Expected checksum '.checkSum($payload).', got '.$sum);
    }
    my $type = substr($payload,0,1);
    my $data = substr($payload,1);

    return ($type,$data);
}

=head2 downloadLog

Download from internal data logger

=cut
sub downloadLog {
    sendRawPacket(PID_LOG_DATA_ADDR,"");
    my ($type,$data) = readPacket();
    if($type ne PID_DATA){
        print STDERR "Got no address data\n";
        return undef;
    }
    my ($start_addr, $a, $b, $end_addr) = unpack("i*",$data);

    my $data = '';

    my $read  = 0;    # bytes already read
    my $max   = $end_addr - $start_addr;   # maximum bytes to read
    my $addr  = $start_addr;

    while($read < $max){
        my $toread = 4*4*1024;
        $toread = ($max - $read) if( ($max - $read) < $toread);

        # request data
        my $msg = pack("V",$addr+$read).pack("v",$toread)."\x00";
        sendRawPacket(PID_READ_TRACKPOINTS,$msg);


        # read answer
        my ($type,$logdata) = readPacket();
        if($type ne PID_DATA){
            # something went wrong
            print STDERR "Did not receive expected Trackpoint data";
            return 0;
        }
        $data .= $logdata;
        #print OUT $data;

        # increase read counter
        $read += $toread;

        # send ACK
        sendRawPacket(PID_ACK,"");
    }

    createGPX($read, $data);
    return 1;
}


=head2 downloadInfo

Reads general information from the device and returns it as hash

=cut
sub downloadInfo {
    sendRawPacket(PID_QRY_INFORMATION,"");
    my ($type,$data) = readPacket();
    if($type ne PID_DATA){
        print STDERR "Got no info data\n";
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

    $info{'username'} =~ s/\0.*$//;
    return %info;
}

=head2 downloadFWInfo

Reads the firmware information

=cut
sub downloadFWInfo {
    sendRawPacket(PID_QRY_FW_VERSION,'');
    my ($type,$data) = readPacket();
    if($type ne PID_DATA){
        print STDERR "Got no info data\n";
        return undef;
    }
    $data=~s/,.*$//;
    return $data;
}

=head2 downloadWaypointData

Reads all Waypoints from the device and prints it as GPX data

=cut
sub downloadWaypointData {
    my $dogpx = shift(); # should a GPX be created? (or do we just need waypoint names)
    my @wpnames;

    # check for available waypoints
    my %info = downloadInfo();
    if(!defined(%info) || !$info{'waypoints'}){
        if($dogpx){
            nicedie("There are no waypoints available");
        }else{
            return @wpnames;
        }
    }

    my ($type,$data);
    my $read   = 0;                  # waypoints alreay read
    my $max    = $info{'waypoints'}; # waypoints to read
    my $points = '';

    while($read < $max){
        my $toread = 32;
        $toread = ($max - $read) if( ($max - $read) < $toread);

        # request data
        my $msg = pack("V",$read).pack("v",$toread)."\x01";
        sendRawPacket(PID_QRY_WAYPOINTS,$msg);

        # read answer
        ($type,$data) = readPacket();
        if($type ne PID_DATA){
            # something went wrong
            print STDERR "Did not receive expected waypoint data";
            return 0;
        }
        $points .= $data;

        # increase read counter
        $read += $toread;
    }

    # Create GPX file
    if($dogpx){
        print OUT "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        print OUT "<gpx version=\"1.0\" creator=\"navilink\"
                    xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"
                    xmlns=\"http://www.topografix.com/GPX/1/0\"
                    xsi:schemaLocation=\"http://www.topografix.com/GPX/1/0 http://www.topografix.com/GPX/1/0/gpx.xsd\">\n";
    }

    # now parse the waypoints
    for(my $i=0; $i<$read; $i++){
        my @tp = unpack("vva7CllvC6CCCC",substr($points,$i*32,32));

        #print hexdump(substr($points,$i*32,32))."\n";
        #print Dumper(\@tp);

        $tp[2] =~ s/[\x00\s]+$//;
        if($dogpx){
            printf OUT "<wpt lat=\"%f\" lon=\"%f\">\n", $tp[4]/10000000, $tp[5]/10000000;
            printf OUT "  <name>%s</name>\n", $tp[2];
            printf OUT "  <ele>%f</ele>\n", $tp[6]*0.3048; #feet to meters
            printf OUT "  <sym>%s</sym>\n", waypointSymbol($tp[13]);
            printf OUT "  <time>%04d-%02d-%02dT%02d:%02d:%02d</time>\n", $tp[7]+2000, $tp[8], $tp[9], $tp[10], $tp[11], $tp[12];
            printf OUT "</wpt>\n";
        }else{
            $wpnames[$tp[1]] = $tp[2];
        }
    }

    if($dogpx){
        print OUT "</gpx>\n";
        return 1;
    }else{
        return @wpnames;
    }
}

=head2 downloadTrackData

Reads all Trackpoints from the device and prints it as GPX data

=cut
sub downloadTrackData {
    my %info = downloadInfo();
    if(!defined(%info) || !$info{'trackpoints'}){
        nicedie("There are no trackpoints available");
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

    # Create GPX file
    createGPX($read, $track);

    return 1;
}

sub createGPX {
    my ($read,$track) = @_;

    # Create GPX file
    print OUT "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    print OUT "<gpx version=\"1.0\" creator=\"navilink\"
                xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"
                xmlns=\"http://www.topografix.com/GPX/1/0\"
                xsi:schemaLocation=\"http://www.topografix.com/GPX/1/0 http://www.topografix.com/GPX/1/0/gpx.xsd\">\n";
    print OUT "<trk>\n";
    print OUT "<trkseg>\n";

    # now parse the track log
    for(my $i=0; $i<$read; $i+=32){
        my @tp = unpack("vvVVVVvCCCCCCCCCC",substr($track,$i,32));
        for (my $j=2; $j<6; $j++) {
            if ($tp[$j]>0x7FFFFFFF) { $tp[$j]=$tp[$j]-0x80000000-0x80000000; }
        }

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
}

=head2 deleteTrackData

Deletes all trackpoints from the device

=cut
sub deleteTrackData {
    my %info = downloadInfo();
    if(!defined(%info) || !$info{'trackpoints'}){
        nicedie("There are no trackpoints available");
    }
    my $addr  = $info{'trackbuffer'};        # buffer address

    dowait('All track data will be deleted from the device!');
    my $msg = pack("V",$addr)."\x00\x00\x00";
    sendRawPacket(PID_ERASE_TRACK,$msg);

    my ($type,$data) = readPacket();
    nicedie("Trackpoint deletion failed.") if($type ne PID_CMD_OK);

    if ($info{'trackpoints'} > 4096){
        $msg = pack("V",$addr + (4096 * 32))."\x00\x00\x00";
        sendRawPacket(PID_ERASE_TRACK,$msg);

        ($type,$data) = readPacket();
        nicedie("Trackpoint deletion failed.") if($type ne PID_CMD_OK);
    }
}

=head2 deleteAllWaypointData

Deletes all waypoints from the device

=cut
sub deleteAllWaypointData {
    my %info = downloadInfo();
    if(!defined(%info) || !$info{'waypoints'}){
        nicedie("There are no waypoints available");
    }

    dowait('All waypoints will be deleted from the device!');
    my $msg = "\x00\xf0\x00\x00";
    sendRawPacket(PID_DEL_ALL_WAYPOINT,$msg);

    my ($type,$data) = readPacket();
    nicedie("Waypoint deletion failed.") if($type ne PID_ACK);
}

=head2 deleteWaypointData

Deletes the waypoints given in the input GPX from the device. (Based on the
waypoint names, not the coordinates).

=cut
sub deleteWaypointData {
    my @wpnames = @_;            # existing waypoints
    nicedie("No deletable waypoints on the device") if(!scalar(@wpnames));

    my @data = parseGPX('wpt');
    nicedie("Could not read any waypoints to delete") if(! scalar(@data) );

    my $max     = scalar(@data); # waypoints to delete

    dowait("Attempting to remove $max waypoints from the device!");

    my $del=0;
    for(my $i=0; $i<$max; $i++){
        my ($id) = grep $wpnames[$_] eq $data[$i]->{name}, 0..$#wpnames;
        if($id){
            $msg  = "\x00\x00";
            $msg .= pack('v',$id);
            sendRawPacket(PID_DEL_WAYPOINT,$msg);

            my ($type,$data) = readPacket();
            if($type ne PID_ACK){
                printf STDERR "removal of waypoint '%s' failed - maybe in use?\n", $data[$i]->{name};
            }else{
                $del++;
            }
        }else{
            printf STDERR "waypoint '%s' not found on the device\n", $data[$i]->{name};
        }
    }
}


=head2 uploadWaypointData

Uploads waypoint read from the input GPX file

=cut
sub uploadWaypointData {
    my @data = parseGPX('wpt');
    nicedie("Could not read any waypoints to upload") if(! scalar(@data) );

    my $max     = scalar(@data); # waypoints to upload
    my @wpnames = @_;            # existing waypoints

    for(my $i=0; $i<$max; $i++){
        if (grep $data[$i]->{name} eq $_, @wpnames){
            printf STDERR "skipped existing waypoint '%s'\n",$data[$i]->{name};
            next;
        }


        $msg  = "\x00\x40\x00\x00";
        $msg .= pack('a6',$data[$i]->{name});
        $msg .= "\x00\x00";
        $msg .= pack('l',int($data[$i]->{lat}*10000000));
        $msg .= pack('l',int($data[$i]->{lon}*10000000));
        $msg .= pack('v',int($data[$i]->{ele}/0.3048)); #feet to meters
        $msg .= pack('C6',$data[$i]->{time});
        $msg .= pack('C',$data[$i]->{sym});
        $msg .= "\x00\x7e";

        sendRawPacket(PID_ADD_A_WAYPOINT,$msg);

        my ($type,$data) = readPacket();
        if($type ne PID_DATA){
            printf STDERR "upload of waypoint %s, %s (%s) failed\n",
                   $data[$i]->{lat}, $data[$i]->{lon}, $data[$i]->{name};

        }
    }
}

=head2 parseGPX I<type>

Parses a GPX file into a data structure. This is not a real XML parser!

=cut
sub parseGPX {
    my $type = shift;
    my @data;

    my $gpx = join('',<IN>);
    while($gpx =~ s/<$type\s([^>]+)>(.*?)<\/$type>//is){
        my $point = $1.' '.$2;
        my %pt;

        if($point =~ m/lat="(-?\d+(\.\d+)?)"/is){
            $pt{lat} = $1;
        }
        if($point =~ m/lon="(-?\d+(\.\d+)?)"/is){
            $pt{lon} = $1;
        }
        next if(!$pt{lat} || !$pt{lon});

        if($point =~ m/<ele>\s*(-?\d+(\.\d+)?)\s*<\/ele>/is){
            $pt{ele} = $1;
        }
        if($point =~ m/<time>\s*(\d\d\d\d)-(\d\d)-(\d\d)T(\d\d):(\d\d):(\d\d).*?<\/time>/is){
            $pt{time} = [$1 - 2000,$2,$3,$4,$5,$6];
        }
        if($point =~ m/<sym>(.+?)<\/sym>/is){
            my $sym  = $1;
            $pt{sym} = waypointSymbol($sym);
        }
        if($point =~ m/<name>(.+?)<\/name>/is){
            $pt{name} = uc($1);
            $pt{name} =~ s/[^A-Z0-9]+//is;
        }

        # strip GC prefix from geocache names (we only have 6 chars)
        if(lc($sym) == 'geocache'){
            $pt{name} =~ s/^GC//;
        }

        push(@data,\%pt);
    }

    return @data;
}

=head2 waypointSymbol I<lookup>

Translates a symbol number to a descriptive name. The names are should be close
to the ones used by Garmin, but this is not always possible.

If a name instead of a number is given, the number for the given name is looked up

See http://home.online.no/~sigurdhu/MapSource-text.htm for Garmin symbol names

=cut
sub waypointSymbol {
    my $lookup = shift();

    my @symbols = (
        'Waypoint',
        'Flag',
        'Residence',
        'Waypoint',
        'Electricity Pylon',
        'Tunnel',
        'Gas Station',
        'Convenience Store',
        'Short Tower',
        'Summit',
        'Swimming Area',
        'Forest',
        'Bridge',
        'Crossing',
        'Fork',
        'Right Turn',
        'Left Turn',
        'Bird',
        'Lodging',
        'Campground',
        'Radio Beacon',
        'Cable Car',
        'Church',
        'Tall Tower',
        'Skiing Area',
        'Marina',
        'Fish',
        'Hunting Area',
        'Lake',
        'Fishing Area',
        'Lighthouse',
        'Beach',
        'Boat Ramp',
        'Bike',
        'Railway',
        'Money',
        'Truck Stop',
        'Scenic Area',
        'Gas Station 2',
        'Bar',
        'Runway',
        'Airport',
        'Medical Facility',
        'Hotel',
        'Parking Area'
    );

    # was a number given?
    return $symbols[$lookup] if($lookup =~ m/^\d+$/);

    # "fix" Geocaches
    $lookup =~ s/Geocache/Flag/i;

    # still here? Find the number for a symbol
    for(my $i=0; $i<$#symbols; $i++){
        if(uc($symbols[$i]) eq uc($lookup)){
            return $i;
        }
    }

    # still here? We found nothing, return standard
    return 0;
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

=head2 wait

Print a warning message and wait 8 seconds, displaying a counter.
Gives the user the chance to abort dangerous commands

=cut
sub dowait {
    my $msg = shift;
    print STDERR $msg."\n";
    print STDERR "Press CTRL-C to abort.\n";
    for(my $i=8; $i>0; $i--){
        print STDERR "$i ";
        sleep(1);
    }
    print STDERR "\n";
}

=head2 nicedie I<message>

Print a message and exit

=cut
sub nicedie {
    my $msg = shift();
    print STDERR "$msg\n";

    $DEV->close if $DEV;
    close(IN) if IN;
    close(OUT) if OUT;
    exit 1;
}

=head2 help

Just print the usage screen

=cut
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

  info    print number of waypoints, routes and trackpoints
  gettp   download track data as GPX
  deltp   delete all track data from the device
  getwp   download waypoints as GPX
  putwp   upload waypoints given as GPX
  updwp   update waypoints with given GPX (ignore existing ones)
  delwp   delete ALL waypoints from the device
  remwp   remove waypoints given as GPX from the device
  getlog  download internal log data as GPX (BGT/GT-31 only)
EOT

    exit 0
}

###############################################################################
# main


# prepare options
getopts('d:hvqi:o:',\%OPT);
$OPT{'d'} = '/dev/ttyUSB0' if(!$OPT{'d'});

help() if($OPT{'h'} || scalar(@ARGV)!=1);

# open device
$DEV = new Device::SerialPort ($OPT{'d'}) || nicedie("Can't open ".$OPT{'d'}.": $^E");
$DEV->baudrate(115200);
$DEV->databits(8);
$DEV->parity("none");
$DEV->stopbits(1);

# sync
sendRawPacket(PID_SYNC,"");
my ($type,$data) = readPacket();
nicedie("Failed to start the communication. Did you enable the NaviLink mode?")
    if($type ne PID_ACK);

# open files
if($OPT{'i'}){
    open(IN,$OPT{'i'}) || nicedie("Could not open ".$OPT{'i'}." for reading");
}else{
    *IN = *STDIN;
}

if($OPT{'o'}){
    open(OUT,">".$OPT{'o'}) || nicedie("Could not open ".$OPT{'o'}." for writing");
}else{
    *OUT = *STDOUT;
}



# handle commands
if($ARGV[0] eq 'gettp'){
    downloadTrackData();
}elsif($ARGV[0] eq 'deltp'){
    deleteTrackData();
}elsif($ARGV[0] eq 'getwp'){
    downloadWaypointData(1);
}elsif($ARGV[0] eq 'putwp'){
    uploadWaypointData();
}elsif($ARGV[0] eq 'updwp'){
    uploadWaypointData(downloadWaypointData(0));
}elsif($ARGV[0] eq 'remwp'){
    deleteWaypointData(downloadWaypointData(0));
}elsif($ARGV[0] eq 'delwp'){
    deleteAllWaypointData();
}elsif($ARGV[0] eq 'getlog'){
    downloadLog();
}elsif($ARGV[0] eq 'info'){
    my %info  = downloadInfo();
    my $fwver = downloadFWInfo();
    print OUT 'username    : '.$info{'username'}."\n";
    print OUT 'firmware    : '.$fwver."\n";
    print OUT 'serial      : '.$info{'serial'}."\n";
    print OUT 'protocol    : '.$info{'protocol'}."\n";
    print OUT 'waypoints   : '.$info{'waypoints'}."\n";
    print OUT 'routes      : '.$info{'routes'}."\n";
    print OUT 'trackpoints : '.$info{'trackpoints'}."\n";
}else{
    printf STDERR "Unknown command '%s'. Nothing done.\n", $ARGV[0];
}


# quit if wanted
sendRawPacket(PID_QUIT,"") if ($OPT{'q'});

$DEV->close;
close(IN);
close(OUT);
exit 0;

