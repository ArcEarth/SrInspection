@pbrt %1 --outfile %1.exr
@exrtotiff %1.exr %1.tiff
@del %1.exr