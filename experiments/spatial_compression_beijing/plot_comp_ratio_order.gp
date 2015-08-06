set term postscript eps 35 enhanced color solid
set style fill solid 1.00 border
set style data histogram
set style histogram cluster gap 1
set encoding iso_8859_1
set output "comp_ratio_order_beijing.eps"
set xlabel "order"
set ylabel "compression ratio"
set key top left
set ytics 4
set yrange[0:20]
set boxwidth .9 absolute
plot "PPM_order.dat" using 2:xtic(1) notitle with linespoints lt 1 lc 1 lw 4 pt 2 ps 4
#, "short_path_freq_subt_length_subt_order.dat" using 2:xtic(1) notitle with linespoints lt 1 lc 3 lw 4 pt 7 ps 4

