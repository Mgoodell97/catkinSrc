clear
kk = 3
Z = dlmread('map'+string(kk) +'.txt',' ');
size_z = size(Z);
    Z = Z(:, 1: size_z(2) - 1);
    mat_size = size(Z);
    x = ( (1:mat_size(1)));
    y = ( (1:mat_size(2)));
    map = Z;
    map_size = size(map);
    for ii = 1:map_size(1)
        for jj = 1:map_size(2)
            temp = map(ii,jj);
            if temp > 30
                map(ii,jj) = .55;
            elseif temp >= 0
                map(ii,jj) = 1;
            else
                map(ii,jj) = .89 ;
            end
        end
    end
    
     
    %map = transpose(map);
    

    %% add plan
    
    sw_ne = dlmread(string(kk)+'.txt',' ');
    
    P = dlmread('plan'+string(kk)+'.txt',' ');
    curr_P = [P(:,1) - sw_ne(1), P(:,2) - sw_ne(2)]
    for ii = 1:length(P)
        map(curr_P(ii,1), curr_P(ii,2)) = .68;
    end
    %%
    
    wp = dlmread('wp'+string(kk)+'.txt',' ');
    curr_wp = [wp(:,1) - sw_ne(1), wp(:,2) - sw_ne(2)]
    for ii = 1:length(wp)
        map(curr_wp(ii,1), curr_wp(ii,2)) = .9;
    end
    

    %%
    map = map -1;
    surf(y,x,map, 'EdgeColor','none');
    %pcolor(map)
    axis equal
    grid on
    caxis([-1 0])
    colorbar; colormap('colorcube')
    xlabel ('x (m)'); ylabel ('y (m)'); zlabel('map color')
    view(-90, 90);
    hold on
    %saveas(gcf,strcat(num2str(kk) , '.png'))
    pause(1)
    hold off
         %% add visited
     V = dlmread('visited'+ string(kk)+ '.txt',' ');
     curr_V = [V(:,1) - sw_ne(1), V(:,2) - sw_ne(2)];
     hold on
     plot(curr_V(:,1), curr_V(:,2),  '.')
     
     
     
     
     
     
     
     
     
     
     
     
     