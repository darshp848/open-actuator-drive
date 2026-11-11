"""Separate validation, motion, contact, and rendering processes for MATLAB."""
import argparse
import json
import shutil
import sys
import traceback
import robot


def stage(name, output):
    if name == 'validate':
        output.mkdir(parents=True, exist_ok=False)
        from validate_mechanics import validate
        from build_contact_fixture import build
        validate(robot.ROOT / 'results/mechanics.json')
        build()
        for filename in ('mechanics.json', 'fixture.json'):
            shutil.copy2(robot.ROOT / 'results' / filename, output / filename)
        sources = {str(p.relative_to(robot.ROOT)): robot.sha256(p)
                   for p in [*robot.ROOT.glob('*.py'), *robot.ROOT.glob('*.m'), robot.ROOT/'models/gripper_fixture.xml',
                             robot.ROOT/'assets-manifest.json']}
        (output/'source-manifest.json').write_text(json.dumps(sources, indent=2))
        return {'passed': True}
    for filename, expected in json.loads((output/'source-manifest.json').read_text()).items():
        assert robot.sha256(robot.ROOT/filename) == expected, f'Source changed since validation: {filename}'
    robot.verify_assets()
    assert json.loads((output/'mechanics.json').read_text())['passed']
    if name == 'motion':
        from run_diagnostics import run
        return run(output/'motion')
    if name == 'contact':
        assert json.loads((output/'motion/result.json').read_text())['passed']
        from run_contacts import run
        return run(output/'contact')
    import mujoco as mj
    from PIL import Image
    model, data = robot.load(contact=True)
    camera = mj.MjvCamera()
    camera.lookat[:] = [.20, 0., .48]
    camera.distance, camera.azimuth, camera.elevation = 1.55, 140., -18.
    model.vis.global_.offwidth, model.vis.global_.offheight = 1000, 800
    with mj.Renderer(model, height=800, width=1000) as renderer:
        renderer.update_scene(data, camera=camera)
        Image.fromarray(renderer.render()).save(output/'preview.png')
    return {'passed': True, 'preview': str(output/'preview.png'), 'pose': 'home_open_grippers'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', choices=['validate', 'motion', 'contact', 'render'], required=True)
    parser.add_argument('--output', type=robot.Path, required=True)
    args = parser.parse_args()
    try:
        result = stage(args.stage, args.output.resolve())
    except Exception:
        if args.output.is_dir():
            (args.output/f'{args.stage}-failure.txt').write_text(traceback.format_exc())
        raise
    (args.output/f'{args.stage}-result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({'stage': args.stage, 'passed': result['passed'], 'python': sys.version}))
