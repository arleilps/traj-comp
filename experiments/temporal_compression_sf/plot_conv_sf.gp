set term postscript eps 40 enhanced color solid
set style fill solid 1.00 border
set style data histogram
set style histogram cluster gap 1
set encoding iso_8859_1
set output "conv_sf.eps"
set xlabel "iteration"
set ylabel "log-likelihood"
set key top left
set xtics 4
set yrange [0:]
set format y '%.1e'
set pointsize 2
plot "conv_sf.dat" using 1:2 notitle with points lt 1 lc 3 lw 3 pt 7

