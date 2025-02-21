#!/bin/sh
# Bash script + XSLT that splits .test entries into subsets according to the following.
split () {
xsltproc --param lth $1 --param rth $2 split.xslt fcsmap.test > fcsmap-$1-$2.test
}

split 0 99
split 100 199
split 200 299
split 300 399
split 400 499
split 500 599
split 600 699
split 700 799
split 800 899
split 900 999
#
split 1000 1099
split 1100 1199
split 1200 1299
split 1300 1399
split 1400 1499
split 1500 1599
split 1600 1699
split 1700 1799
split 1800 1899
split 1900 1999
#
split 2000 2099
split 2100 2199
split 2200 2299
split 2300 2399
split 2400 2499
split 2500 2599
split 2600 2699
split 2700 2799
split 2800 2899
# split 2900 2999
#
# split 3000 3099
# split 3100 3199
# split 3200 3299
# split 3300 3399
# split 3400 3499
# split 3500 3599
# split 3600 3699
