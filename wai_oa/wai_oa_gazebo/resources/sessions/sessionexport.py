#! /usr/bin/env python3

from __future__ import annotations
import argparse
import sys
import tempfile
import uno
from pathlib import Path
from ooodev.draw import ImpressDoc
from ooodev.draw import DrawPage
from ooodev.loader import Lo
from ooodev.utils.file_io import FileIO
from ooodev.utils.images_lo import ImagesLo
from ooodev.utils.type_var import PathOrStr
from ooodev.draw.filter.export_jpg import ExportJpg
from ooodev.draw.filter.export_png import ExportPng
from com.sun.star.text import XText
from com.sun.star.beans import PropertyValue
from com.sun.star.util import XCloseable



class Slide2Image:
    def __init__(self,fnm: PathOrStr,idx: int,img_fmt: str,out_dir: PathOrStr = "",resolution: int = 1280) -> None:
        _ = FileIO.is_exist_file(fnm, True)
        self._fnm = FileIO.get_absolute_path(fnm)
        if out_dir:
            _ = FileIO.is_exist_dir(out_dir, True)
            self._out_dir = FileIO.get_absolute_path(out_dir)
        else:
            self._out_dir = Path(tempfile.mkdtemp())
        if idx < 0:
            print("Index is less then zero. Initializing and creating preview!")
        self._idx = idx
        self._img_fmt = img_fmt.strip()
        self._resolution = resolution

    def main(self) -> None:
        load = Lo.Loader(Lo.ConnectPipe(headless=False))
        components = Lo.get_desktop().getComponents()
        docs = components.createEnumeration()
        file_url = FileIO.fnm_to_url(self._fnm)
        doc=None
        doc_was_open=False
        for document in docs:
            if document.Location == file_url:
                print("Document is already open, using existing one: "+document.Location)
                doc = ImpressDoc(document)
                doc_was_open=True
                break
        if doc==None:
            print("Document not opened, reopening!")
            doc=ImpressDoc.open_doc(fnm=self._fnm)
            doc_was_open=False

        # Configure mime type for compression
        # names = ImagesLo.get_mime_types()
        # Lo.print("Known GraphicExportFilter mime types:")
        # for name in names:
        #     print(f"  {name}")
        mime = ImagesLo.change_to_mime(self._img_fmt)
        # optionally set filter data to change resolution and have finer control.

        # Create session preview with reduced resolution
        dt = None

        # If session is initialized (Slide 0 and Index -1) then:
        # - Create scene previews in preview subfolder 
        # - Export global session config from MASTER SLIDE NOTES into session.yaml 
        if self._idx==-1:
            if mime == "image/jpeg":
                # width, height = ImagesLo.get_dpi_width_height(width=slide.component.Width,height=slide.component.Height,resolution=self._resolution,)
                dt = ExportJpg(
                    color_mode=True,
                    pixel_width=320,
                    pixel_height=180,
                    quality=80,
                    logical_width=320,
                    logical_height=180,
                ).to_filter_dict()
            if mime == "image/png":
                # width, height = ImagesLo.get_dpi_width_height(width=slide.component.Width,height=slide.component.Height,resolution=self._resolution,)
                # note: if `translucent=True` then page image are not exported,
                # also page margins are not exported.
                dt = ExportPng(
                    pixel_width=320,
                    pixel_height=180,
                    logical_width=320,
                    logical_height=180,
                    compression=1,
                    translucent=False,
                    interlaced=False,
                ).to_filter_dict()
            counter=0
            for slide in doc.slides:
                # Extract all slides as images for preview
                out_fnm=self._out_dir / f"preview/scene_{counter+1}.{self._img_fmt}"
                slide.save_page(fnm=out_fnm, mime_type=mime, filter_data=dt)
                # Extract all notes as scene_X.yaml files for preview
                slide = doc.slides[counter]
                notes_page = slide.get_notes_page()
                shapes = notes_page.get_shapes_collection()
                for shp in shapes:
                    try:
                        xtext = Lo.qi(XText, shp)
                    except Exception as e:
                        xtext = None
                    if xtext:
                        s = xtext.getString()
                        if "teleprompter:" in s:
                            out_preview_fnm=self._out_dir / f"preview/scene_{counter+1}.yaml"
                            with open(out_preview_fnm,"w") as text_file:
                                text_file.write(s)
                # Increment scene counter
                counter=counter+1
            # Extract global session config ([slide count 0] / [index -1]) from Master Slide Notes
            notes = []
            master_page=doc.get_master_page(0) # Extract from first slide!
            master_page_notes=master_page.get_notes_page() # Extract slide count max, even if no notes are available.
            with open(Path(__file__).parent/ "session.yaml","w") as text_file:
                text_file.write("setup_session_scene_count_max: "+str(doc.slides.get_count())+"\n")
                for shape in master_page_notes: # If Master Slide Notes contain a valid default session config, then extract...
                    if hasattr(shape, 'Text'):
                        text = shape.Text.getString()
                        if "setup_session_defaults" in text:
                            text_file.write(text) # If Master Slide Notes are empty, OA loads a default configuration!
        else:
            slide = doc.slides[self._idx]
            notes_page = slide.get_notes_page()      
            shapes = notes_page.get_shapes_collection()
            for shp in shapes:
                try:
                    xtext = Lo.qi(XText, shp)
                except Exception as e:
                    xtext = None
                # Extract the actual string from XText object
                if xtext:
                    s = xtext.getString()
                    if "teleprompter:" in s:
                        with open(Path(__file__).parent/ "session.yaml","w") as text_file:
                            # text_file.write("setup_scene_"+str(self._idx+1)+":\n    ")
                            # text_file.write(s.replace("\n","\n    "))
                            text_file.write(s)
            # Extract slide as image
            res_width=self._resolution;
            res_height=int(res_width*9/16);
            if mime == "image/jpeg":
                dt = ExportJpg(
                    color_mode=True,
                    pixel_width=res_width,
                    pixel_height=res_height,
                    quality=80,
                    logical_width=res_width,
                    logical_height=res_height,
                ).to_filter_dict()
            if mime == "image/png":
                dt = ExportPng(
                    pixel_width=res_width,
                    pixel_height=res_height,
                    logical_width=res_width,
                    logical_height=res_height,
                    compression=1,
                    translucent=False,
                    interlaced=False,
                ).to_filter_dict()
            out_fnm = self._out_dir / f"scene.{self._img_fmt}"
            print(f'Saving scene with index {self._idx} to "{out_fnm}"')
            slide.save_page(fnm=out_fnm, mime_type=mime, filter_data=dt)

        if doc_was_open == False:
            print("Closing document, after reopening!")
            doc.close()
            Lo.close_office()


def main() -> int:
    parser = argparse.ArgumentParser(description="sceneexport")
    parser.add_argument(
        "-i",
        "--infile",
        help="File path of input file",
        action="store",
        dest="file_path",
        required=True,
    )
    parser.add_argument(
        "-x",
        "--index",
        help="Optional index of slide to convert to image. Default: %(default)i",
        action="store",
        dest="idx",
        type=int,
        default=0,
    )
    parser.add_argument(
        "-o",
        "--outfolder",
        help="Optional output Directory. Defaults to temporary dir sub folder.",
        action="store",
        dest="out_dir",
        default=str(Path(__file__).parent/ ""),
    )
    parser.add_argument(
        "-f",
        "--format",
        help="Extension of the converted file. Default: %(default)s",
        action="store",
        dest="output_format",
        default="png",
    )
    parser.add_argument(
        "-r",
        "--resolution",
        help="Optional image output resolution width. Defaults to 1280 pixels in a 16:9 format.",
        action="store",
        dest="resolution",
        default=1280,
        type=int,
    )

    # read the current command line args
    if len(sys.argv) == 1:
        parser.print_help()
        return 0
    args = parser.parse_args()
    sl = Slide2Image(fnm=args.file_path,idx=args.idx,img_fmt=args.output_format,out_dir=args.out_dir,resolution=args.resolution)
    sl.main()

    return 0



if __name__ == "__main__":
    SystemExit(main())



