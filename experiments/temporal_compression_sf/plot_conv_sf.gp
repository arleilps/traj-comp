set term postscript eps 40 enhanced color solid
set style fill solid 1.00 border
set style data histogram
set style histogram cluster gap 1
set encoding iso_8859_1
set output "conv_sf.eps"
set xlabel "iteration"
set ylabel "log-likelihood"
set key top left
set xtic 1
set yrange [0:]
set format y '%.1e'
set ytics 40000000
set pointsize 2
plot "conv_sf.dat" using 1:2 notitle with linespoints lt 1 lc 1 lw 4 pt 2 ps 3

