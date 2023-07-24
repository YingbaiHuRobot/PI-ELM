function [x2, y2, p] = DTW(x, y, w)
% Trajectory realignment through dynamic time warping.
% x= r(1).Data，作为模板
% y= s(n).Data， 需要处理数据
% w=50,时间窗长度
if nargin<3
  w = Inf;
end

sx = size(x,2);%数据长度
sy = size(y,2);

w = max(w,abs(sx-sy)); 

%Initialization
D = ones(sx+1,sy+1) * Inf; 
D(1,1) = 0;

%DP loop
for i=1:sx%类似网格遍历了
  for j=max(i-w,1):min(i+w,sy)
    D(i+1,j+1) = norm(x(:,i)-y(:,j)) + min([D(i,j+1), D(i+1,j), D(i,j)]);
  end
end

i=sx+1; j=sy+1;
p=[];
while i>1 && j>1
 [~,id] = min([D(i,j-1), D(i-1,j), D(i-1,j-1)]);%id表示属于哪一个
 if id==1
   j=j-1;
 elseif id==2
   i=i-1;
 else
   i=i-1;
   j=j-1;
 end
 p = [p, [i;j]];%i,j的值存进去
end

p = fliplr(p(:,1:end-1)-1);%将数组从左向右翻转

x2 = x(:,p(1,:));
y2 = y(:,p(2,:));

%Resampling
x2 = spline(1:size(x2,2), x2, linspace(1,size(x2,2),sx));
y2 = spline(1:size(y2,2), y2, linspace(1,size(y2,2),sx));
