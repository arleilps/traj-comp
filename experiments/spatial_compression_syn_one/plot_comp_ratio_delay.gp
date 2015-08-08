set term postscript eps 35 enhanced color solid
set style fill solid 1.00 border
set style data histogram
set style histogram cluster gap 1
set encoding iso_8859_1
set output "comp_ratio_delay_syn_one.eps"
set xlabel "delay (min)"
set ylabel "compression ratio"
set key top left
set ytics 20
set yrange [0:80]
set logscale x 2
set boxwidth .9 absolute
plot "ontrac_delay.dat" using 1:2 title "ONTRAC" with linespoints lt 1 lc 1 lw 4 pt 2 ps 3, "press_delay.dat" using 1:2 title "PRESS" with linespoints lt 1 lc 3 lw 4 pt 7 ps 3
