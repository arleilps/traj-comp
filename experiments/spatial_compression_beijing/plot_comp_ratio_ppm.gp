set term postscript eps 30 enhanced color solid
set style fill solid 1.00 border
set style data histogram
set style histogram cluster gap 1
set encoding iso_8859_1
set output "comp_ratio_ppm.eps"
set xlabel "order"
set ylabel "compression ratio"
set key top left
set ytics 2
set yrange[0:12]
set boxwidth .9 absolute
plot "PPM_order.dat" using 2:xtic(1) title 'train-train' lc 2 fs pattern 3,"PPM_order.dat" using 3 title 'train-test' lc 1 fs pattern 1

