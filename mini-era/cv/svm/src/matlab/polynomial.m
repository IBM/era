function ret = polynomial(d, a, b, dim)

bt = transpose(b);
ret=(a*bt)^d/dim;

