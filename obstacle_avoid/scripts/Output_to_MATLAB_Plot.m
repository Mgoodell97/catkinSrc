clc
clear all
list = dir;
list_of_names = [];
for ii = 1:length(list)
    if exist(string(ii)+'.txt') == 0
        gg = ii - 1;
        break
    end
end
 figure
 pause(2)
%%
for kk = 0:gg
    Z = dlmread('map'+string(kk) +'.txt',' ');
    Z = Z(:, 1: size(Z,2));
    mat_size = size(Z);
    x = ( (1:mat_size(1)));
    y = ( (1:mat_size(2)));
    map = Z;
    
    for ii = 1:size(map,2)
        for jj = 1:size(map,1)
            temp = map(jj,ii);
            if temp > 25
                map(jj,ii) = .55;
            elseif temp >= 0
                map(jj,ii) = 1;
            else
                map(jj,ii) = .89 ;
            end
        end
    end
    
    P = dlmread('plan'+string(kk)+'.txt',' ');
    for ii = 1:length(P)
        map(P(ii,1), P(ii,2)) = .68;
    end
    
    P = dlmread('wp'+ string(kk)+'.txt',' ');
    size_P = size(P)
    for ii = 1:size_P(1)
        length_limit = length(P)
        
        map(P(ii,1), P(ii,2)) = .13;
    end
    map = map -1;
    map = transpose(map);
    
    
    
    % for ii = 1:length(traj)
    %     map(round(traj(ii,2)*2), round(traj(ii,1)*2)) = .3;
    % end
    
    V = dlmread('visited'+ string(kk)+ '.txt',' ');
    
    
   
    surf(x,y,map, 'EdgeColor','none');
    %pcolor(map)
    axis equal
    grid on
    caxis([-1 0])
    colorbar; colormap('colorcube')
    xlabel ('x (m)'); ylabel ('y (m)'); zlabel('map color')
    view(-90, 90);
    hold on
    
    plot(V(:,1), V(:,2),  '.')
    colorbar('off')
    drawnow
    
    %saveas(gcf,strcat(num2str(kk) , '.png'))
    pause(1)
    hold off
end
%%
f = figure
Z = dlmread('map'+string(kk) +'.txt',' ');
Z = Z(:, 1: size(Z,2));
mat_size = size(Z);
x = ( (1:mat_size(1)));
y = ( (1:mat_size(2)));
map = Z;

for ii = 1:size(map,2)
    for jj = 1:size(map,1)
        temp = map(jj,ii);
        if temp > 20
            map(jj,ii) = .55;
        elseif temp >= 0
            map(jj,ii) = 1;
        else
            map(jj,ii) = .95 ;
        end
    end
    
end
map = map - 1;
%map = transpose(map) - 1;
surf(y,x,map, 'EdgeColor','none');
%pcolor(map)
axis equal
grid on
caxis([-1 0])
colorbar; colormap('colorcube')
xlabel ('x (m)'); ylabel ('y (m)'); zlabel('map color')
view(0, 90);
hold on
traj = dlmread('traj.txt',' ');
traj_small = [];
for ii = 1:length(traj)
    if mod(ii,40) == 0
        traj_small = [traj_small; traj(ii,3:4)];
    end
end
    sw_ne = dlmread(string(kk)+'.txt',' ');
    traj_test = [traj_small(:,2)*5 - sw_ne(2), traj_small(:,1)*5 - sw_ne(1)]

 plot(traj_test(:,1), traj_test(:,2), 'b', 'LineWidth', 2)
ax = gca;
for ii = 1:length(ax.XTickLabel)
    ax.XTickLabel{ii} = num2str(str2num(ax.XTickLabel{ii})/5);
end
for ii = 1:length(ax.YTickLabel)
    ax.YTickLabel{ii} = num2str(str2num(ax.YTickLabel{ii})/5);
end
colorbar('off')
hold on

%saveas(gcf,strcat(num2str(kk) , 'last.png'))
