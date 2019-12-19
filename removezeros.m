function [match3d_1, match3d_2, aux1, aux2] = removezeros(xyz, matchpoints1, matchpoints2, imgot, imget)
    nonzeros = 0;
    for i=1:length(matchpoints1)
        if xyz{imgot}(round(matchpoints1.Location(i,2)), round(matchpoints1.Location(i,1)), 3)
            if xyz{imget}(round(matchpoints2.Location(i,2)), round(matchpoints2.Location(i,1)), 3)
                nonzeros=nonzeros+1;
            end
        end
    end
    a=0;
    match3d_1 = zeros(nonzeros, 3);
    match3d_2 = zeros(nonzeros, 3);
    aux1 = zeros(nonzeros, 2);
    aux2 = zeros(nonzeros, 2);
    for i=1:length(matchpoints1)
        if xyz{imgot}(round(matchpoints1.Location(i,2)), round(matchpoints1.Location(i,1)), 3)
            if xyz{imget}(round(matchpoints2.Location(i,2)), round(matchpoints2.Location(i,1)), 3)
                a = a + 1;
                aux1(a, :) = matchpoints1.Location(i, :);
                aux2(a, :) = matchpoints2.Location(i, :);
                match3d_1(a, :) = xyz{imgot}(round(matchpoints1.Location(i, 2)), round(matchpoints1.Location(i, 1)), :);
                match3d_2(a, :) = xyz{imget}(round(matchpoints2.Location(i, 2)), round(matchpoints2.Location(i, 1)), :);
            end
        end
    end
end