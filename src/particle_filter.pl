#!/usr/bin/perl 


print "update_count\tgt_x\tgt_y\tgt_th\tbest_x\tbest_y\tbest_th\thighw\tavgw\tavgdiff\tpred_adj_x\tpred_adj_y\tpred_adj|th\n";

while (<>) {

	#print $_;
	next if ( /best_particle_association/ );
	if (/Prediction avg adj: (.*)$/) {
		$avgpredadj = $1;
	}
	if (/Average diff: (.*)$/) {
		$avgdiff = $1;
	}
	if (/Update count: (.*)$/) {
		$updatecount = $1;
	}
	if (/highest w: (.*)$/) {
		$highw = $1;
	}
	if (/average w: (.*)$/) {
		$avgw = $1;
	}
	if (/Ground Truth\t(.*)$/) {
		$ground = $1;
	}
	if (/best_particle(.*$)/) {
		#print "XXX";
		$best = $1;
		print "$updatecount\t$ground$best\t$highw\t$avgw\t$avgdiff\t$avgpredadj\n";
	}
}
