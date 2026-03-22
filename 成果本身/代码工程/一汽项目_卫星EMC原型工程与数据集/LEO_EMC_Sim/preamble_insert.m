function y = preamble_insert(payloadSym)
cfg = evalin('base','cfg');

payloadSym = payloadSym(:);
L = cfg.phy.preambleHalfLen;
reps = cfg.phy.preambleReps;

persistent p0;
if isempty(p0) || numel(p0) ~= L
    rng(1);
    b = randi([0 1], 2*L, 1);
    b0 = b(1:2:end);
    b1 = b(2:2:end);
    I = 1 - 2*b0;
    Q = 1 - 2*b1;
    p0 = (I + 1j*Q)/sqrt(2);
end

preamble = repmat(p0, reps, 1);
payloadLen = cfg.phy.frameBits;
preambleLen = L * reps;

y = complex(zeros(preambleLen + payloadLen, 1));
y(1:preambleLen) = preamble;
y(preambleLen+1:end) = payloadSym(1:payloadLen);
end
