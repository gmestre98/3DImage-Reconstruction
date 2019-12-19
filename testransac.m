P1 = rand(3, 50);
[U, S, V] = svd(rand(3));
R = U*V';
while(det(R) < 0)
    [U, S, V] = svd(rand(3));
    R = U*V';
end
T = rand(3,1);
a = ones(1, length(P1));
P2 = R*P1 + T*a;

[in1, in2, a, b] = ransac(P1', P2', P1', P2');
P3 = P2;
outliers = [1 3 5 22 42];
P3(3, outliers) = 45;
[ina1, ina2, a, b] = ransac(P1', P3', P1', P3');