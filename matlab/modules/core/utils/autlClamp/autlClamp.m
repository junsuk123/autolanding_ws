function y = autlClamp(x, lo, hi)
% autlClamp
% Clamp numeric value to [lo, hi].

if ~isfinite(lo)
    lo = -inf;
end
if ~isfinite(hi)
    hi = inf;
end
y = min(max(x, lo), hi);
end
