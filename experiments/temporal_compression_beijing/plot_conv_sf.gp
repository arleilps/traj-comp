set term postscript eps 35 enhanced color solid
set style fill solid 1.00 border
set style data histogram
set style histogram cluster gap 1
set encoding iso_8859_1
set output "conv_sf.eps"
set xlabel "iteration"
set ylabel "likelihood"
set key top left
#set ytics 4
set yrange[:0]
#set boxwidth .9 absolute
set xtics 20
plot "conv_sf.dat" using 1:2 notitle with lines lt 1 lc 1 lw 3

