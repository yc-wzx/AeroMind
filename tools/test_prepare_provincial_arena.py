import copy, json, tempfile, unittest
from pathlib import Path
import numpy as np
from PIL import Image
from prepare_provincial_arena import ROOT, build, validate

class PreparationTests(unittest.TestCase):
    def setUp(self): self.source=json.loads((ROOT/'tools/config/provincial_arena_input_template.json').read_text())
    def test_unknown_dimension_rejected(self):
        self.source['field']['start']['x']=None
        with self.assertRaises(ValueError):validate(self.source)
    def test_wrong_units_rejected(self):
        self.source['units']='mm'
        with self.assertRaises(ValueError):validate(self.source)
    def test_unresolved_geometry_rejected(self):
        self.source['unresolved_dimensions']=['corridor width']
        with self.assertRaises(ValueError):validate(self.source)
    def test_zero_wall_rejected(self):
        self.source['field']['collision_segments'][0]=[0,0,0,0]
        with self.assertRaises(ValueError):validate(self.source)
    def test_unconfirmed_competition_input_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'source.json';p.write_text(json.dumps(self.source))
            with self.assertRaises(ValueError):build(p,Path(tmp)/'out',competition=True)
            self.assertFalse((Path(tmp)/'out').exists())
    def test_provisional_and_mirrored_layouts(self):
        with tempfile.TemporaryDirectory() as tmp:
            for label in ('original','mirror'):
                source=copy.deepcopy(self.source)
                if label=='mirror':
                    f=source['field'];w=f['arena']['width']
                    for key in ('start','shooting_zone','target_zone'):f[key]['x']=w-f[key]['x']
                    for rect in f['course_surfaces']:rect['x']=w-rect['x']
                    f['reference_route']=[[w-x,y] for x,y in f['reference_route']]
                    f['collision_segments']=[[w-a,b,w-c,d] for a,b,c,d in f['collision_segments']]
                path=Path(tmp)/(label+'.json');path.write_text(json.dumps(source));out=Path(tmp)/label
                result=build(path,out);self.assertTrue(result['all_pass']);self.assertFalse(result['competition_arena_verified'])
                if label=='original':
                    self.assertTrue(np.array_equal(np.asarray(Image.open(out/'arena.pgm')),np.asarray(Image.open(ROOT/'src/uav_bringup/maps/provincial_2025_provisional.pgm'))))
                with self.assertRaises(FileExistsError):build(path,out)

if __name__=='__main__':unittest.main()
