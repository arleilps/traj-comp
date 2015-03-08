set term postscript eps 30 enhanced color solid
set style fill solid 1.00 border
set style data histogram
set style histogram cluster gap 1
set encoding iso_8859_1
set output "comp_ratio_train_train.eps"
set xlabel "training rate (%)"
set ylabel "compression ratio"
set key top left
set ytics 2
set yrange[0:9]
set boxwidth .9 absolute
plot "FS_training_rate.dat" using 2:xtic(1) title 'FS' lc 2 fs pattern 3,"SP_training_rate.dat" using 2 title 'SP' lc 3 fs pattern 4, "SPFS_training_rate.dat" using 2 title 'PRESS' lc 0 fs pattern 5, "PPM_training_rate.dat" using 2 title 'PPM' lc 1 fs pattern 1

