function [gradD, lossD] = dGradients(netD, xReal, xFake)
    pReal = forward(netD, xReal);
    pFake = forward(netD, xFake);
    epsv = 1e-6;
    lossD = -mean(log(pReal+epsv) + log(1-pFake+epsv));
    gradD = dlgradient(lossD, netD.Learnables);
end
