function kalman_enlarge_frame_left(dim);

drawnow;
main_data = get(gcf,'UserData');
set(gcf,'UserData',main_data);

axes_hidden       = findobj(gcf,'Tag','demo_kalman_axes_hidden');
axes_top          = findobj(gcf,'Tag','demo_kalman_axes_top');
axes_bottom       = findobj(gcf,'Tag','demo_kalman_axes_bottom');
pos_axes_top      = get(axes_top,'Position');
pos_axes_bottom   = get(axes_bottom,'Position');
pos_axes_hidden   = get(axes_hidden,'Position');

if strcmp('off',get(axes_hidden,'Visible'));
    for k = dim.main_fig.left_frame.width.small:5:dim.main_fig.left_frame.width.large
        set(axes_top,'Position',[dim.main_fig.space_lr+dim.main_fig.left_frame.width.small+dim.main_fig.axes.space_left+k-dim.main_fig.left_frame.width.small pos_axes_top(2) dim.main_fig.width-(2*dim.main_fig.space_lr)-dim.main_fig.left_frame.width.small-dim.main_fig.axes.space_left-k+dim.main_fig.left_frame.width.small pos_axes_top(4)]);
        set(axes_bottom,'Position',[dim.main_fig.space_lr+dim.main_fig.left_frame.width.small+dim.main_fig.axes.space_left+k-dim.main_fig.left_frame.width.small pos_axes_bottom(2) dim.main_fig.width-(2*dim.main_fig.space_lr)-dim.main_fig.left_frame.width.small-dim.main_fig.axes.space_left-k+dim.main_fig.left_frame.width.small pos_axes_bottom(4)]);        
        if (k > dim.main_fig.left_frame.width.small + 2*dim.main_fig.space_lr + 10)
            set(axes_hidden,'Visible','On');
            set(axes_hidden,'Position', [pos_axes_hidden(1) pos_axes_hidden(2), k-dim.main_fig.left_frame.width.small-(2*dim.main_fig.space_lr), pos_axes_hidden(4)]);
        end;
        drawnow;        
    end;
    tag_axes_hidden = get(axes_hidden,'Tag');
    set(gcf,'CurrentAxes',axes_hidden);
    im = imread('kalman_structure.jpg','jpg');
    image(im);
    grid off;
    axis tight;
    set(gca,'XTickLabel','','YTickLabel','');
    set(gca,'XTick',[]);
    set(gca,'YTick',[]);
    set(gca,'Tag',tag_axes_hidden);    
end;