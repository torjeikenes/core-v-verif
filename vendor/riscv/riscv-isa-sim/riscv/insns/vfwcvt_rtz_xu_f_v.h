// vfwcvt.rtz,xu.f.v vd, vs2, vm
VI_VFP_WCVT_FP_TO_INT(
  {;},                                                      // BODY8
  { vd = f16_to_ui32(vs2, softfloat_round_minMag, true); }, // BODY16
  { vd = f32_to_ui64(vs2, softfloat_round_minMag, true); }, // BODY32
  {;},                                                      // CHECK8
  { require_extension(EXT_ZFH); },                          // CHECK16
  { require_extension('F'); },                              // CHECK32
  uint                                                      // sign
)
