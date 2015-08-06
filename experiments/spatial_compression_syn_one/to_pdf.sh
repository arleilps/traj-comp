for f in $(ls *.eps)
do
	epstopdf $f
done
