function imgOut = makeSTFTImageSimple(r, stft, imgSize)
    [S,~,~] = spectrogram(r, stft.win, stft.overlap, stft.nfft, stft.fs, 'centered');
    P = abs(S).^2;
    img = 10*log10(P + 1e-12);
    img = img - min(img(:));
    img = img ./ (max(img(:)) + 1e-12);
    imgOut = imresize(img, imgSize);
end
