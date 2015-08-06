set term postscript eps 35 enhanced color solid
set style fill solid 1.00 border
set style data histogram
set style histogram cluster gap 1
set encoding iso_8859_1
set output "comp_time_order.eps"
set ylabel "time (seg)"
set key top right
set format y "10^{%L}"
set log y
set xtics ("training" 0, "compression" 1)
set boxwidth .75 absolute
set xrange[-0.5:1.5]
plot "PPM_order.dat" using 2 title 'ONTRAC' lc 1, "short_path_freq_subt_length_subt_order.dat" using 2 title 'PRESS' lc 3

