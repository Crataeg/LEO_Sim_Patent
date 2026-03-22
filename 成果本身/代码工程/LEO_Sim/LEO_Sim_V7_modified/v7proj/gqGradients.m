function [gradG, gradQ, lossG, lossInfo] = gqGradients(netG, netD, netQ, zc, cTrue, infoLambda)
    xFake = forward(netG, zc);
    pFake = forward(netD, xFake);
    cPred = forward(netQ, xFake);

    epsv = 1e-6;
    lossAdv  = -mean(log(pFake+epsv));
    lossInfo = mean((cPred - cTrue).^2,'all');

    lossG = lossAdv + infoLambda*lossInfo;

    gradG = dlgradient(lossG, netG.Learnables);
    gradQ = dlgradient(lossInfo, netQ.Learnables);
end
