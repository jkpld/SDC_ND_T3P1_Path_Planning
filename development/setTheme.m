function setTheme(fig,darkLight)
% SETTHEME Set a figure theme to light or dark.
%
% setTheme(fig,darkLight)
%
% Input parameters:
% fig : The handle of the figure to modify
% darkLight : string with the value 'dark' or 'light'.
%
% James Kapaldo

switch darkLight
    case 'dark'
        f = @(x) x;
        axisBackground = 0.2*[1 1 1];
    case 'light'
        f = @(x) 1-x;
        axisBackground = [1 1 1];
    otherwise
        error('figureTheme:unknownThemeName','Unknown theme name.')
end

% Add a short pause so that everything draws correctly.
pause(1e-5);


try
    if strcmp(fig.Type,'figure')
        fig.Color = f(0)*[1 1 1];
    end
catch
end

childs = [findall(fig,'type','text');...
    findall(fig,'Style','text');...
    findall(fig,'type','textbox');...
    findall(fig,'type','axes');...
    findall(fig,'type','colorbar');...
    findall(fig,'type','uiflowcontainer');...
    findall(fig,'type','uicontainer'); ...
    findall(fig,'type','uigridcontainer'); ...
    findall(fig,'type','uicontrol');...
    findall(fig,'type','legend')];

for i = 1:length(childs)
    
    h = childs(i);
    
    if strcmp(h.Tag, 'ignore')
        continue;
    end
    
    try
        switch h.Style
            case 'text'
                h.ForegroundColor = f(0.6)*[1 1 1];
                h.BackgroundColor = f(0)*[1 1 1];
            case 'textbox'
                h.Color = f(0.6)*[1 1 1];
            case {'radiobutton','checkbox'}
                h.BackgroundColor = f(0.1)*[1 1 1];
        end
    catch 
    end
    
    try
        switch h.Type
            case 'text'
                h.Color = f(0.6)*[1 1 1];
            case {'TextBox', 'textboxshape'}
                h.Color = f(0.6)*[1 1 1];
            case 'axes'
                try
                    h.Backdrop.FaceColor = axisBackground;
                    h.GridColor = f(0.5)*[1 1 1];
                    h.YColor = f(0.6)*[1 1 1];
                    h.XColor = f(0.6)*[1 1 1];
                    h.ZColor = f(0.6)*[1 1 1];
                    h.Title.Color = f(0.9)*[1 1 1];
                    h.Box = 'off';
                    try
                        h.XRuler.TickDir = 'out';
                        h.YRuler.TickDir = 'out';
                    catch
                        h.TickDir = 'out';
                    end

                    h.XGrid = 'on';
                    h.YGrid = 'on';
                    h.ZGrid = 'on';
                catch
                end
                
            case 'colorbar'
                
                h.Ruler.Color = f(0.6)*[1 1 1];
                h.BoxHandle.Visible = 'off';
                h.TickLength = 0.01;
                try
                    h.TickDirection = 'out';
                catch
                    h.Ruler.TickDir = 'out';
                end
                h.Ruler.Axle.Visible = 'off';
                
            case {'uiflowcontainer','uicontainer','uigridcontainer'}
                h.BackgroundColor = f(0.1)*[1 1 1];
            case 'legend'
                
                set(h.ItemText,'Color',f(0.8)*[1 1 1]);
                h.TextColor = f(0.8)*[1 1 1];
                h.Color = f(0)*[1 1 1];
                    
        end
    catch ME
        rethrow(ME)
    end
    
end

% Add a short pause so that everything draws correctly.
pause(1e-5);

end