function [A_d, A_x0] = polyeval(a,x0)
% POLYEVAL Evaluate all derivatives of a polynomial at the point x0 and
% also return the coefficients of the polynomial with origin translated to
% x0.
%
% [A_d, A_x0] = polyeval(A, x0)
%
% Input :
% A - 1 x Na array giving the coefficients of a polynomial such that
%   A(x) = A(1) + A(2)*x + A(3)*x^2 ... = A(i+1)*x^(i). 
%   Note this is the opposite ordering as Matlabs polynomials that use
%   polyval.
%
% Output :
% A_d - numel(x0) x Na matrix where the i,j'th entry gives the
%   (j-1)'th derivative evaluated at x0(i).
% A_x0 - numel(x0) x Na matrix where each row gives the new
%   coefficients of the polynomial relative to x0. That is
%
%   A_x0(i,j) x^j = A_j (x + x0(i))^j
%
% Note that A_x0(i,:) = A_d(i,:) ./ factorial(0:numel(A));
%
% If you only need the value of the polynomial at x0, then it is faster to
% just use polyval(flip(A),x0).
%
% Theory:
%
% A(x) = a_j x^j = b_j (x-x0)^j
% A(x+x0) = a_j (x+x0)^j = b_j x^j
%
% (The above are using implicit summation over j.) Now solve for b_j using
% binomial theorem. The derivatives of A at x0 are then simply
%
% d^nA/dx^n (x0) = b_j * j!

% James Kapaldo
n = numel(a);
Nx = numel(x0);

if isrow(x0)
    x0 = x0';
end

% factorial (0:n-1)!
fac = [1, 1, cumprod(2:n-1)];

% x0^j / j!
x0 = [ones(Nx,1),cumprod(ones(1,n-1).*x0, 2)] ./ fac;

% a(j)*j!
f = a.*fac;

A_d = zeros(Nx,n);

for r=0:(n-1)
    % bp(r+1) = s_r(r,x0,a,n,fac) % derivatives 0:n-1
    % b(r+1) = bp(r+1) / fac(r+1), polynomial coefficients
    
    j = r:n-1;
    A_d(:,r+1) = sum(f(j+1).*x0(:,j-r+1),2);
end

if nargout > 1
    A_x0 = A_d ./ fac;
end

end


% function s = s_r(r,x0,a,n, fac)
% 
% f = a.*fac;
% s = f(r+1);
% 
% for j = (r+1):(n-1)
%     s = s + f(j+1)*x0./fac(j - r + 1);
%     x0 = x0*x0;
% end
% 
% end
