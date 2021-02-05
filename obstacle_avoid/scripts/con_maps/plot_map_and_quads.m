close all
load("d_plume1/Con_XYPlane_Z=1.5_T=60.mat")
max_pos = [20,46];
C = transpose(data);
time = 600;

% Load Data
%load('Hestia.mat')
%load('Run2.mat')
[rowLenY, colLenX] = size(C);
[colsX, rowsY] = meshgrid(1:colLenX, 1:rowLenY);

% Used when referring to matrix
rowMin = 1;
rowMax = rowLenY;
colMin = 1;
colMax = colLenX;

% Used when referring to x/y directions
xMin = 1;
xMax = colLenX;
yMin = 1;
yMax = rowLenY;

% Plot Initial
% figure('Name', 'Release Area', 'units', 'normalized', 'outerposition', [0 0 1 1]);
figure('Name', 'Release Area');
hold on
C(isnan(C))=0;
pcolor(colsX,rowsY, C);
xlim([1 colLenX]);
ylim([1 rowLenY]);
shading flat

%%

date = '2018-5-31';
list = dir(['~/Dropbox/pso_data/'  date '*']);

for ii = 1:size(list,1) - 1
    U1 = csvread(['~/Dropbox/pso_data/' list(ii).name '/a1']);
    U2 = csvread(['~/Dropbox/pso_data/' list(ii).name '/a2']);
    U3 = csvread(['~/Dropbox/pso_data/' list(ii).name '/a3']);
    
    r = 3;
    if length(U1) < .9 * time
        continue
    end
    
    for jj=1:length(U1)
        if norm(U1(jj,4:5) - max_pos) < r
            t1 = jj;
            break
        end
        t1 = length(U1);
    end
    
    % plot con
    % figure('Name', 'Release Area', 'units', 'normalized', 'outerposition', [0 0 1 1]);
    figure('Name', [list(ii).name + "  ii:" + num2str(ii)]);
    title("convergence in " + num2str(t1/50) + "min")
    hold on
    C(isnan(C))=0;
    pcolor(colsX,rowsY, C);
    xlim([1 colLenX]);
    ylim([1 rowLenY]);
    shading flat
    % Plot traj
    plot(U1(:,1), U1(:,2))
    plot(U2(:,1), U2(:,2))
    plot(U3(:,1), U3(:,2))
    rectangle('Position',[10,11,14,15],'FaceColor',[0 0 0],'EdgeColor','k','LineWidth',1)
    rectangle('Position',[27,16,10,6],'FaceColor',[0 0 0],'EdgeColor','k','LineWidth',1)
    rectangle('Position',[40,10,10,5],'FaceColor',[0 0 0],'EdgeColor','k','LineWidth',1)
    rectangle('Position',[40,23,10,6],'FaceColor',[0 0 0],'EdgeColor','k','LineWidth',1)
    rectangle('Position',[55,10,10,6],'FaceColor',[0 0 0],'EdgeColor','k','LineWidth',1)
    rectangle('Position',[55,23,10,5],'FaceColor',[0 0 0],'EdgeColor','k','LineWidth',1)
    rectangle('Position',[43,32,7,11],'FaceColor',[0 0 0],'EdgeColor','k','LineWidth',1)
    rectangle('Position',[41,43,5,6],'FaceColor',[0 0 0],'EdgeColor','k','LineWidth',1)
    rectangle('Position',[25,39,10,6],'FaceColor',[0 0 0],'EdgeColor','k','LineWidth',1)
    rectangle('Position',[25,52,6,10],'FaceColor',[0 0 0],'EdgeColor','k','LineWidth',1)
    rectangle('Position',[39,57,10,6],'FaceColor',[0 0 0],'EdgeColor','k','LineWidth',1)
    rectangle('Position',[58,51,6,11],'FaceColor',[0 0 0],'EdgeColor','k','LineWidth',1)
end