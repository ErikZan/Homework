file = "plot.csv"

set ylabel "value (mm)"
set xlabel "Frequency (s)"
set grid
plot file using 1:2 with lines title "x",\
 file u 1:3 w l t "xm" ,\
 file u 1:4 w l t "xx"