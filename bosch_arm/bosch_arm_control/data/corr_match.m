%vert=importdata('vert_3dps_flt.txt','\t');
%horiz=importdata('horiz_3dps_flt.txt','\t');
seg=vert(2001:7000,17);
seg=seg-mean(seg);
corr_max=0;
i_max=1;
for i=1001:3000
    ft=horiz(i:i+4999,17);
    ft=ft-mean(ft);
    corr=seg'*ft;
    if corr_max<corr
        corr_max=corr;
        i_max=i;
    end
end
i_max