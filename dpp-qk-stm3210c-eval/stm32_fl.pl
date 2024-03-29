#!/usr/bin/perl

use Net::Telnet;
use Cwd 'abs_path';

my $numArgs = $#ARGV + 1;
if($numArgs != 1) {
    die( "Usage ./stm32_flash.pl [main.bin] \n");
}

my $file = abs_path($ARGV[0]);

my $ip = "127.0.0.1";   # localhost
my $port = 4444;

my $telnet = new Net::Telnet (
    Port   => $port,
    Timeout=> 30,
    Errmode=> 'die',
    Prompt => '/>/');

$telnet->open($ip);

print $telnet->cmd('reset');
print $telnet->cmd('halt');
print $telnet->cmd('stm32f1x mass_erase 0');
print $telnet->cmd('flash write_bank 0 '.$file.' 0');
print $telnet->cmd('reset');
print $telnet->cmd('exit');

print "\n";
