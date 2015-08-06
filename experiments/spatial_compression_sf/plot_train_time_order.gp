set term postscript eps 35 enhanced color solid
set style fill solid 1.00 border
set style data histogram
set style histogram cluster gap 1
set encoding iso_8859_1
set output "train_time_order.eps"
set xlabel "order"
set ylabel "training time (seg)"
set key top left
set log y
#set yrange[0:12]
set boxwidth .9 absolute
plot "PPM_order.dat" using 4:xtic(1) notitle with linespoints lt 1 lc 1 lw 4 pt 2 ps 4, "short_path_freq_subt_length_subt_order.dat" using 4:xtic(1)  notitle with linespoints lt 1 lc 3 lw 4 pt 7 ps 4

