// vsrl.vi vd, vs2, zimm5
VI_VI_ULOOP
({
  vd = vs2 >> (zimm5 & (sew - 1) & 0x1f);
})
