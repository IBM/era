#!/bin/perl

$n_in  = 0;
$n_out = 0;
$check_now = 0;
$line_np = 0;
$test_num = 1;

for ($i = 0; $i < 6; $i++)  {
    $differ[$i] = 0;
}

while(<>) {
    chop;
    if ($_ =~ m/^INPUTS/) {
	$n_in = 1;
    } elsif ($_ =~ m/OUTPUTS/) {
	$n_out = 1;
    } else {
	if ($n_in == 1) {
	    @f_in = split(/,/,$_);
	    $n_in = 2;
	} elsif ($n_out == 1) {
	    @f_out = split(/,/,$_);
	    $n_out = 2;
	    $check_now = 1;
	}
    }
    if ($check_now == 1) {
	$check_now = 0;
	if (($n_in != 2) || ($n_out != 2)) {
	    print STDERR "ERROR: LINE $line_no : n_in = $n_in and n_out = $n_out\n";
	}
	for ($i = 2; $i < 6; $i++)  {
	    for ($j = 0; $j < 64; $j++) {
		$iidx = 64*$i+$j;
		$oidx = 64*($i-2)+$j;
		if ($f_in[$iidx] != $f_out[$oidx]) {
		    if ($differ[$i] == 0) {
			$differ[$i] = $test_num;
			printf " %2u : %u vs %u : %u SO differ[%u] %u\n", $iidx, $f_in[$iidx], $f_out[$oidx], ($f_in[$iidx] != $f_out[$oidx]), $i, $test_num;
		    }
		}
	    }
	}
	$n_in = 0;
	$n_out = 0;
	$test_num++;
    }
    $line_no++;
}

print "RESULTS:\n";
print " d_brtab27 : differ = $differ[0]\n";
print "   symbols : differ = $differ[1]\n";
print "       mm0 : differ = $differ[2]\n";
print "       mm1 : differ = $differ[3]\n";
print "       pp0 : differ = $differ[4]\n";
print "       pp1 : differ = $differ[5]\n";

