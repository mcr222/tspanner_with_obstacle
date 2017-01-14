fid = fopen('data.txt');
n = fgetl(fid);
n_obs = fgetl(fid);
ratio = fgetl(fid);


n = str2num(n);
n_obs = str2num(n_obs);
ratio = str2num(ratio);

data = zeros([2,n]);
data1 = zeros([2,n_obs]);



for i=1:n
    temp = fgetl(fid);
    temp = str2num(temp);
    data(1,i) = temp(1);
    data(2,i) = temp(2);
end

for i=1:n_obs
    temp = fgetl(fid);
    temp = str2num(temp);
    data1(1,i) = temp(1);
    data1(2,i) = temp(2);
end
data1(1,n_obs+1) = data1(1,1);
data1(2,n_obs+1) = data1(2,1);
x = data1(1,:);
y = data1(2,:);
plot(x,y,'-o');
hold on;
scatter(data(1,:),data(2,:));
fclose(fid);