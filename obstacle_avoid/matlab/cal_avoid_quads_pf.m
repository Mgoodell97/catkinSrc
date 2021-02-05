M = 0:.1:10;
s = 5;
a = 1;
sum_inv = s./(M.^a) - .5

plot(M,sum_inv)
axis([0 10 0 3]) 