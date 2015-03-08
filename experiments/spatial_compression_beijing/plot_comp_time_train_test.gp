set term postscript eps 35 enhanced color solid
set style fill solid 1.00 border
set encoding iso_8859_1
set output "comp_time_train_test.eps"
set xlabel "training rate (%)"
set ylabel "time (sec.)"
set key top right
set ytics 10
plot "FS_training_rate.dat" using 5:xtic(1) title 'FS' with linespoints lt 1 lc 0 lw 4 pt 2 ps 4, "SP_training_rate.dat" using 5 title 'SP' with linespoints lt 1 lc 2 lw 4 pt 5 ps 4,"SPFS_training_rate.dat" using 5 title 'PRESS' with linespoints lt 1 lc 3 lw 4 pt 7 ps 4,"PPM_training_rate.dat" using 5 title 'PPM' with linespoints lt 1 lc 1 lw 4 pt 9 ps 4

