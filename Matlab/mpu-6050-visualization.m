%% ========================================================================
%  ROCKET + LIVE VISUALIZATION  -  STM32C092RC + MPU-6050
%  Incoming CSV: ax,ay,az,gx,gy,gz,roll100,pitch100
% ========================================================================

clear; clc; close all;

PORT  = "COM3";      % find it with serialportlist("available")
BAUD  = 115200;
TAU   = 0.03;        % smoothing [s] (jittery: 0.08 / laggy: 0.03)
RSIGN = +1;          % -1 if roll turns the wrong way
PSIGN = +1;          % -1 if pitch tilts the wrong way
M     = 300;         % samples kept in the plots

s = serialport(PORT, BAUD, "Timeout", 1);
flush(s);

%% ---------------- FIGURE ------------------------------------------------
fig = figure('Name','Roket','Color','k','NumberTitle','off', ...
             'Position',[200 100 1180 720],'Renderer','opengl');

%% ---------------- LEFT: 3D SCENE ---------------------------------------
ax = axes('Parent',fig,'Color','k','Position',[0.01 0.03 0.60 0.80]);
hold(ax,'on'); grid(ax,'on');
axis(ax,'equal'); axis(ax,'vis3d');
xlim(ax,[-3 3]); ylim(ax,[-3 3]); zlim(ax,[-3 3]);
view(ax,-40,16);
ax.XColor=[.45 .45 .5]; ax.YColor=[.45 .45 .5]; ax.ZColor=[.45 .45 .5];
ax.GridColor=[.35 .35 .4]; ax.GridAlpha=0.3;
camlight(ax,'headlight'); lighting(ax,'gouraud'); material(ax,'dull');

% Fixed horizontal reference (NOT attached to the transform)
plot3(ax,[-3 3],[0 0],[0 0],'-','Color',[.3 .32 .38],'LineWidth',1);
plot3(ax,[0 0],[-3 3],[0 0],'-','Color',[.3 .32 .38],'LineWidth',1);

% --- Rocket: long and thin, body axis +Z, centre ~ origin ---
tf = hgtransform('Parent',ax);
N  = 16;
R  = 0.25;                          % body radius (thin)
Z0 = -2.40;  ZB = 1.30;  ZN = 2.60; % base / top of body / nose tip -> length 5.0

[cx,cy,cz] = cylinder(R,N);
h(1) = surf(cx, cy, Z0+cz*(ZB-Z0),'FaceColor',[.92 .93 .96],'EdgeColor','none');

[nx,ny,nz] = cylinder([R 0],N);
h(2) = surf(nx, ny, ZB+nz*(ZN-ZB),'FaceColor',[.90 .25 .25],'EdgeColor','none');

[tx,ty,tz] = cylinder([0 R],N);
h(3) = surf(tx, ty, Z0+tz*0.02,'FaceColor',[.55 .55 .60],'EdgeColor','none');

zz = [Z0   Z0+0.95  Z0+0.95  Z0];   % 3 fins (scaled to the thin body)
rr = [R    R        0.72     0.52];
for a = [0 120 240]
    h(end+1) = patch(rr*cosd(a), rr*sind(a), zz, ...
                     [.20 .52 .95],'EdgeColor','none'); 
end
set(h,'Parent',tf);

%% ---------------- RIGHT: LIVE PLOTS -------------------------------------
xv = 1:M;  rbuf = zeros(1,M);  pbuf = zeros(1,M);

axR = axes('Parent',fig,'Color','k','Position',[0.68 0.55 0.29 0.30]);
hold(axR,'on'); grid(axR,'on');
plot(axR,[1 M],[0 0],'-','Color',[.35 .35 .4]);
hLR = plot(axR, xv, rbuf,'Color',[.25 .80 1.0],'LineWidth',1.6);
xlim(axR,[1 M]); ylim(axR,[-180 180]);
axR.XTick=[]; axR.YTick=-180:90:180;              % fewer ticks = less drawing work
axR.XColor=[.4 .4 .45]; axR.YColor=[.4 .4 .45]; axR.GridColor=[.3 .3 .35];
title(axR,'ROLL','Color',[.25 .80 1.0],'FontName','Consolas','FontSize',12);

axP = axes('Parent',fig,'Color','k','Position',[0.68 0.13 0.29 0.30]);
hold(axP,'on'); grid(axP,'on');
plot(axP,[1 M],[0 0],'-','Color',[.35 .35 .4]);
hLP = plot(axP, xv, pbuf,'Color',[1.0 .60 .20],'LineWidth',1.6);
xlim(axP,[1 M]); ylim(axP,[-90 90]);
axP.XTick=[]; axP.YTick=-90:45:90;
axP.XColor=[.4 .4 .45]; axP.YColor=[.4 .4 .45]; axP.GridColor=[.3 .3 .35];
title(axP,'PITCH','Color',[1.0 .60 .20],'FontName','Consolas','FontSize',12);

%% ---------------- NUMERIC READOUT --------------------------------------
hTxt = uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.03 0.855 0.45 0.13],'BackgroundColor','k', ...
    'ForegroundColor',[.95 .96 .98],'FontName','Consolas','FontSize',26, ...
    'HorizontalAlignment','left','String',sprintf('ROLL   ---\nPITCH  ---'));

hFps = uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.66 0.925 0.31 0.05],'BackgroundColor','k', ...
    'ForegroundColor',[.45 .50 .58],'FontName','Consolas','FontSize',11, ...
    'HorizontalAlignment','right','String','');

%% ---------------- MAIN LOOP --------------------------------------------
buf  = '';
roll = 0; pitch = 0;
rT   = 0; pT    = 0;
nl   = 0; frames = 0;
tPrev = tic; tRun = tic; tTxt = 0;

try
while ishandle(fig)

    % --- Drain the WHOLE serial buffer, parse ONLY the last line --------
    n = s.NumBytesAvailable;
    if n > 0
        buf = [buf char(read(s,n,"char"))]; 
        e = find(buf==newline,1,'last');
        if ~isempty(e)
            b = find(buf(1:e-1)==newline,1,'last');
            if isempty(b), b = 0; end
            v = sscanf(buf(b+1:e-1),'%f,',8);
            if numel(v)==8
                rT = RSIGN*v(7)*0.01;
                pT = PSIGN*v(8)*0.01;
                nl = nl+1;
            end
            buf = buf(e+1:end);
        end
        if numel(buf)>1024, buf=''; end
    end

    % --- Smoothing independent of frame time ---------------------------
    dt = toc(tPrev); tPrev = tic;
    k  = 1 - exp(-dt/TAU);
    roll  = roll  + k*(mod(rT-roll+180,360)-180);
    pitch = pitch + k*(pT-pitch);

    % --- 3D rotation: Ry(pitch)*Rx(roll), written out ---------------------
    r=roll*pi/180; p=pitch*pi/180;
    cr=cos(r); sr=sin(r); cp=cos(p); sp=sin(p);
    tf.Matrix = [ cp  sp*sr  sp*cr  0
                   0     cr    -sr  0
                 -sp  cp*sr  cp*cr  0
                   0     0      0   1 ];

    % --- Plots: only YData is updated, axes stay fixed -----------------
    rbuf = [rbuf(2:end) roll];
    pbuf = [pbuf(2:end) pitch];
    set(hLR,'YData',rbuf);
    set(hLP,'YData',pbuf);

    frames = frames+1;
    if toc(tRun)-tTxt > 0.12
        set(hTxt,'String',sprintf('ROLL  %+7.2f\nPITCH %+7.2f',roll,pitch));
        set(hFps,'String',sprintf('%.0f fps | %d lines',frames/toc(tRun),nl));
        tTxt = toc(tRun);
    end

    drawnow limitrate
end
catch ME
    disp(ME.message);
end

clear s
disp('Closed.');