# https://stackoverflow.com/questions/11610625/gnuplot-how-to-skip-missing-data-files
file_exists(file) = system("[ -f '".file."' ] && echo '1' || echo '0'") + 0

while ( ! file_exists("data.txt") ){
    print "data.txt is not found"
    pause 1
}

set datafile separator ","

#set yrange [-1:1] 
#set xrange [0:1000]

while(1){
    plot '< tail -n 2000 data.txt' using 3 with line title "CO" lw 2
    pause 0.1
}
