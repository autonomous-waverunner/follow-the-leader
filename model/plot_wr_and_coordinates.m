figure(1);
clf;
hold on;
coordsUTM = [];

[n, ~] = size(coords.Data);
for i = 1 : n
    c = coords.Data(i,:);
    [x, y] = deg2utm(c(2), c(1));
    coordsUTM = [coordsUTM ;[x y] ];
end
plot(wrPos.Data(:,1) - wrPos.Data(1,1), wrPos.Data(:,2) - wrPos.Data(1,2), 'm');
plot(coordsUTM(:,1) - wrPos.Data(1,1), coordsUTM(:,2) - wrPos.Data(1,2), '--b');
% for k = 1:1:size(target.Data,1)-1
%     if target.Data(k,1) > 2
%         plot([wrPos.Data(k,1) target.Data(k,1)],[wrPos.Data(k,2) target.Data(k,2)],'black')
%         plot(target.Data(k,1),target.Data(k,2), 'r*')
%     end
%      if pred_loc.Data(k,1) > 2
%          plot([wrPos.Data(k,1) pred_loc.Data(k,1)],[wrPos.Data(k,2) pred_loc.Data(k,2)],'black')
%          plot(pred_loc.Data(k,1),pred_loc.Data(k,2), 'g*')
%      end
% end


axis equal;
xlim([-20 300]);
ylim([-20 185]);
xticks(-25:25:300);
yticks(0:25:200);
legend({'WR path', 'Reference path'}, 'Location', 'southeast');
%legend({'WR path', 'Reference path'}, 'Location', 'northwest');
