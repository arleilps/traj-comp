set term postscript eps 35 enhanced color solid
set style fill solid 1.00 border
set encoding iso_8859_1
set output "time_ppm.eps"
set xlabel "order"
set ylabel "time (sec.)"
set key top left
set ytics 20
set yrange[0:90]
plot "PPM_order.dat" using 6:xtic(1) title 'training' with linespoints lt 1 lc 0 lw 4 pt 2 ps 4, "PPM_order.dat" using 5 title 'compression' with linespoints lt 1 lc 1 lw 4 pt 9 ps 4

