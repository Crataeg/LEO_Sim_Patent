function r = synthRxSnapshotSimple(className, Ns, snrDb, jsrDb)
    s = qpskRand(Ns);
 
    EsN0 = 10^(snrDb/10);
    noiseVar = 1/EsN0;
    n_awgn = sqrt(noiseVar/2)*(randn(Ns,1)+1j*randn(Ns,1));
 
    cfg = randomJammerCfgSimple(className, jsrDb);
    j = genJammerSimple(cfg, s);
 
    r = s + n_awgn + j;
end
