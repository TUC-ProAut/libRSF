function [] = exportPlot(Handle, Path, Name)
%EXPORTPLOT Export a figure as vector graphic and as rasterized image.
    drawnow update
    pause(0.1);
    
    % concat filname
    FullFile = [Path Name];
    FullFilePDF = [FullFile '.pdf'];
    FullFilePNG = [FullFile '.png'];
    
    if exist('exportgraphics', 'file')
        % new matlab 2020a function
        exportgraphics(Handle, FullFilePDF, 'BackgroundColor', 'none', 'ContentType', 'vector');
        exportgraphics(Handle, FullFilePNG, 'BackgroundColor', 'white');
    elseif exist('export_fig', 'file')
        % you can use export_fig instead (but no transparency):
        export_fig('-painters', '-nofontswap', FullFilePDF, Handle);
        export_fig('-painters', '-nofontswap', FullFilePNG, Handle);
    else
        % fall back using the old print function
        set(Handle,'PaperPositionMode', 'auto');
        set(Handle,'PaperOrientation','landscape');
        print(Handle, FullFile,'-dpng', '-r0');
        print(Handle, FullFile,'-dpdf', '-r0', '-bestfit');
    end   
end
