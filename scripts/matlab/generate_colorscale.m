function generate_colorscale(min_val, max_val, mode)

fontSize = 16;

if (nargin < 2)
    disp('insufficient limits provided: using defaults.');
    min_val = 18.0;
    max_val = 38.0;
end

if (nargin < 3)
    mode = 0;
    disp('no mode provided: assuming thermal only.');
end



im_address = '../output/colorization/frame000000.jpg';

im = imread(im_address);

scale = zeros(size(im), 'uint8');

for iii = 1:size(im,1)
   scale(size(im,1)-iii+1,:,:) = im(iii,:,:); 
end



x=linspace(0,255,2); % 0 to 10 s, 1000 samples
y=linspace(min_val,max_val,3); % 10^1 to 10^3, 1000 samples

H = figure;



set(H, 'Position', [50 50 144 448])

%B = axes;
%set(B,'yaxislocation','right')
%ylabel(B,'Right Label')

set(gca,'LineWidth',0.001)

imagesc(x,y,scale);
set(imgca,'Yscale','linear','Ydir','normal');

if (mode ~= 0)
    L = get(gca,'XLim');
    set(gca,'XTick',[0 255])
else
    L = get(gca,'XLim');
    set(gca,'XTick',[])
end

L = get(gca,'YLim');
rel_range = (max_val - min_val)*0.02;
%set(gca,'YTick',[min_val+rel_range (min_val + max_val)/2 max_val-rel_range])
set(gca,'YTick',[min_val (min_val + max_val)/2 max_val])

old_ticks = get(gca, 'ytick')';




% If 2 decimals needed
if ((rem(10*((max_val-min_val)/2),1) > 0) | (rem(10*((max_val)),1) > 0) | (rem(10*((min_val)),1) > 0))
    if (min_val < 0)
        new_tick_labels = cellfun(@(x) sprintf('%+5.2f',round(x*100)/100), num2cell(old_ticks), 'uniformoutput', false);
    else
        new_tick_labels = cellfun(@(x) sprintf('%5.2f',round(x*100)/100), num2cell(old_ticks), 'uniformoutput', false);
    end

% If 1 decimal needed
elseif ((rem(1*((max_val-min_val)/2),1) > 0) | (rem(1*((max_val)),1) > 0) | (rem(1*((min_val)),1) > 0))
    
    if (min_val < 0)
        %if (max(abs(min_val),abs(max_val)) >= 10)
        %    new_tick_labels = cellfun(@(x) sprintf('%+1.1f',round(x*10)/10), num2cell(old_ticks), 'uniformoutput', false);
        %else
        a = 3
            new_tick_labels = cellfun(@(x) sprintf('%+5.1f',round(x*10)/10), num2cell(old_ticks), 'uniformoutput', false)
        %end
        
    else
        new_tick_labels = cellfun(@(x) sprintf('%5.1f',round(x*10)/10), num2cell(old_ticks), 'uniformoutput', false);
    end
    
% If no decimal needed
else
    if (min_val < 0)
        if (abs(min_val) >= 100)
            new_tick_labels = cellfun(@(x) sprintf('%+5.0f',round(x*10)/10), num2cell(old_ticks), 'uniformoutput', false);
        else
            new_tick_labels = cellfun(@(x) sprintf('%+5.1f',round(x*10)/10), num2cell(old_ticks), 'uniformoutput', false);
        end
        
    else
        new_tick_labels = cellfun(@(x) sprintf('%5.1f',round(x*10)/10), num2cell(old_ticks), 'uniformoutput', false);
    end
end

set(gca, 'yticklabel', new_tick_labels)
    
set(gca,'Ticklength',[0 0])

%set(gca,'yaxislocation','right');

if (mode ~= 0)
    xlabel('Gray');
end

ylabel('Temperature (C)');
%ylabel('Reprojection Error (pixels)');

set(gca,'FontSize', fontSize)
set(gca,'FontName', 'Courier')

if (mode ~= 0)    
    xlhand = get(gca,'xlabel')
    set(xlhand,'fontsize',fontSize)
end
ylhand = get(gca,'ylabel')
set(ylhand,'fontsize',fontSize)

aName = sprintf('../output/scale.pdf')
    
save_to_file_if_possible(H, aName);

bName = sprintf('../output/scale.eps')
%print(H, '-depsc2', bName); % -depsc -depsc2
saveas(H,bName,'eps')

if (mode == 0)
  % system('pdf90 ../output/scale.pdf --outfile ../output/scale.pdf'); 
   
end

topshift = 15;
botshift = -25;
leftshift = 10;
rightshift = 5;

system('convert ../output/scale.pdf ../output/scale.png');
im = imread('../output/scale.png');
im2 = 255*ones([size(im,1)+2-topshift+botshift size(im,2)+2-leftshift+rightshift 3], 'uint8');

for iii = 1+topshift:size(im,1)+min(0,botshift)
    for jjj = 1+leftshift:size(im,2)+min(0,rightshift)
        
        max_k = 1;
        
        if (mode ~= 0)
            max_k = 3;
        end
        
        for kkk = 1:max_k
            im2(iii+1-topshift,jjj+1-leftshift,kkk) = im(iii,jjj,kkk);
        end
    end
end

% Is this the border?
im2(1,:,:) = 0;
im2(size(im2,1),:,:) = 0;
im2(:,1,:) = 0;
im2(:,size(im2,2),:) = 0;
    
%im(1:3,1:3,1:3)
%im2(1:3,1:3,1:3)
%im2(2:end-1, 2:end-1,:) = im(:,:,:);

imwrite(im2, '../output/scale.png', 'png');

end
