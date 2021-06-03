function [A,B] = cutset(initial_state,initial_dia,partition,dim,t)
% % MODIFIED BY NICOLE: accepts n-dim state vector where n>=dim. If n>dim,
% then partitions first dim-dimensions. To update the rest of the values,
% must manually modify code. 
% Also changed diameter to multi-dimension to accommodate for more freedom
% in initial set definition

initial_point = initial_state(1:dim);

if partition == 1
    A = initial_point;
    B = initial_dia;
else
    A = zeros(partition^dim,dim);
    B = repmat(initial_dia/partition,partition^dim,1);
%     B = initial_dia/partition*ones(partition^dim,1);
    for i = 1:dim
        x(i,:) = linspace(initial_point(i)+initial_dia(i),initial_point(i)-initial_dia(i),partition+1);
    end
    for i = 1:partition
        y(:,i) = (x(:,i)+x(:,i+1))/2;
    end


    if dim == 2
        [X1,X2] = ndgrid(y(1,:),y(2,:));
        A = [X1(:),X2(:)];
    elseif dim == 3
        [X1,X2,X3] = ndgrid(y(1,:),y(2,:),y(3,:));
        A = [X1(:),X2(:),X3(:)];
    elseif dim == 4
        [X1,X2,X3,X4] = ndgrid(y(1,:),y(2,:),y(3,:),y(4,:));
        A = [X1(:),X2(:),X3(:),X4(:)];
    elseif dim == 6
        [X1,X2,X3,X4,X5,X6] = ndgrid(y(1,:),y(2,:),y(3,:),y(4,:),y(5,:),y(6,:));
        A = [X1(:),X2(:),X3(:),X4(:),X5(:),X6(:)];
    end
end

%% Edit this if extended state vector variables change
A=horzcat(A,-1*ones(size(A,1),length(initial_state)-dim));
% temp = initial_state(dim+1:end);
for i=1:size(A,1)
    A(i,:) = ARPOD_update(A(i,:),t);
end

