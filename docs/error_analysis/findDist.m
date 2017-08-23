function findDist(img_path)
    img = imread(img_path);
    imshow(img)
    [x,y] = getpts;
    line(x(1:2), y(1:2))
    line(x(3:4), y(3:4),'LineStyle','--')
    m1 = (y(1)-y(2)) / (x(1)-x(2));
    b1 = y(1) - m1*x(1);
    m2 = (y(3)-y(4)) / (x(3)-x(4));
    b2 = y(3) - m2*x(3);
    
    if((y(3)-m1*x(3)-b1)*(y(4)-m1*x(4)-b1)>0)
        disp(polyarea(x([1,2,4,3]),y([1,2,4,3]))/pdist([x(1:2), y(1:2)]))
    else
        2
    end
end