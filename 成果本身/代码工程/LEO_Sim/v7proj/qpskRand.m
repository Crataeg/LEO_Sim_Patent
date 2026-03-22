function s = qpskRand(Ns)
    b = randi([0 3], Ns, 1);
    s = exp(1j*(pi/4 + (pi/2)*double(b)));
end
