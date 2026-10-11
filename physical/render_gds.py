import pya,json
view=pya.LayoutView();idx=view.load_layout(input_gds,0);view.load_layer_props(layer_properties);view.max_hier()
view.set_config('background-color','#ffffff');view.set_config('grid-visible','false');view.set_config('text-visible','false')
for mode in ['overview','routing-detail']:
 for li in list(view.each_layer()):
  q=li;q.visible=False
  layer=q.source_layer
  colors={13:'#c7d0d8',15:'#8399ad',17:'#277da8',19:'#c48135',21:'#8070ad',23:'#4b8c74',25:'#a38891',27:'#777777',29:'#777777'}
  if layer in colors and q.source_datatype==0:
   q.visible=True;q.fill_color=int(colors[layer][1:],16);q.frame_color=q.fill_color;q.dither_pattern=0
  if mode=='routing-detail' and layer==11:q.visible=True;q.fill_color=0xc8c8c8;q.frame_color=q.fill_color
 if mode=='overview':view.zoom_box(pya.DBox(0,0,417.39,417.39))
 else:view.zoom_box(pya.DBox(195,195,225,225))
 view.save_image(output_dir+'/'+mode+'-raw.png',2000,2000)
print(json.dumps(dict(top=view.cellview(idx).layout().top_cell().name,dbu=view.cellview(idx).layout().dbu,modes=['overview M2-M10','detail M1-M10; box 195,195,225,225 um'])))
