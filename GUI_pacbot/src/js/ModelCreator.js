import {
    BoxGeometry,
    Mesh,
    MeshPhongMaterial,
    MeshBasicMaterial,
    Vector3,
    Object3D,
    Group,
    QuadraticBezierCurve3,
    BufferGeometry,
    ConeGeometry,
    Line,
    LineBasicMaterial,
} from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { CSS2DObject } from "three/examples/jsm/renderers/CSS2DRenderer.js";
/* BasePlate Dimensions 6.4 x ?? x 6.4 */
import base_plate_path from "../assets/glb/BasePlate_16x16.glb";
/* Lego Dimensions: (0.8 x 0.48 x 0.8) */
import yumi_path from "../assets/glb/yumi.glb";

class ModelCreator {
    /**********************************************
     * 
     * @param {*} model_path 
     * @param {*} model_containers 
     * @param {*} model_positions 
     * @param {*} model_rotation 
     * @param {*} model_color 
     * @param {*} model_names 
     * @param {*} scene 
     **********************************************/
    constructor(model_path, model_containers,
        model_positions, model_rotation, model_color, model_names, scene) {
        this.model_path = model_path;
        this.model_containers = model_containers;
        this.model_positions = model_positions;
        this.model_rotation = model_rotation;
        this.model_color = model_color;
        this.model_names = model_names;
        this.scene = scene;
        this.loader = new GLTFLoader();
    }
    create = () => {
        this.loadModel();
        this.duplicateModel();
    }

    loadModel = () => {
        const onLoad = (result, model_containers) => {
            const model = result.scene.children[0];
            for (let i = 0; i < model_containers.length; i++) {

                model_containers[i].geometry = model.geometry.clone();
                model_containers[i].material = model.material.clone();
                model_containers[i].scale.set(0.05, 0.05, 0.05);
                model_containers[i].material.color.set(this.model_color)
            }
        };
        this.loader.load(
            this.model_path,
            gltf => onLoad(gltf, this.model_containers),
        ), undefined, function (error) {
            console.log('An error happened: '+error);
        };
    }

    duplicateModel = () => {
        for (let i = 0; i < this.model_containers.length; i++) {

            this.model_containers[i].translateX(this.model_positions[i].x);
            this.model_containers[i].translateY(this.model_positions[i].y);
            this.model_containers[i].translateZ(this.model_positions[i].z);

            if (this.model_rotation) {
                this.model_containers[i].rotateY(Math.PI / 2);
            }

            /* Supplementary Information */
            this.model_containers[i].userData.color = this.model_color;
            this.model_containers[i].name = this.model_names[i];
            this.scene.add(this.model_containers[i]);
        }
    }
}
/************************************************
 * 
 * @param {*} scene 
 ***********************************************/
export function createBase(scene) {
    /* 16x16 ==> 48x24 */
    let bases = [];
    let bases_pos = [];
    let bases_names = [];

    for (let i = 0; i < 6; i++) {
        bases.push(new Mesh());
        bases_names.push("base_" + i);
    }

    const base_pos_0 = new Vector3(0.0, 0.0, 0.0);
    const base_pos_1 = new Vector3(-6.4, 0.0, 0.0);
    const base_pos_2 = new Vector3(6.4, 0.0, 0.0);
    const base_pos_3 = new Vector3(-6.4, 0.0, 3.2);
    const base_pos_4 = new Vector3(0.0, 0.0, 3.2);
    const base_pos_5 = new Vector3(6.4, 0.0, 3.2);

    bases_pos.push(base_pos_0);
    bases_pos.push(base_pos_1);
    bases_pos.push(base_pos_2);
    bases_pos.push(base_pos_3);
    bases_pos.push(base_pos_4);
    bases_pos.push(base_pos_5);

    const baseCreator_ = new ModelCreator(base_plate_path, bases, bases_pos,
        false, 0x004904, bases_names, scene);

    baseCreator_.create()
}
/************************************************
 * 
 * @param {*} pos 
 * @param {*} scene 
 * @param {*} gripper_color 
 ***********************************************/
export function createGripper(scene, pos, gripper_color = 0xbbbbbb) {

    const pos_in_mm = new Vector3(
        pos.y * 0.8 - 8.8,
        pos.z * 0.48,
        pos.x * 0.8 - 2.4
    );

    const gripper = new Group();
    gripper.name = "gripper";
    gripper.position.set(pos_in_mm.x, pos_in_mm.y, pos_in_mm.z);

    const gripper_material = new MeshPhongMaterial({ color: gripper_color, flatShading: true });
    gripper_material.transparent = true;
    gripper_material.shininess = 100;
    gripper_material.opacity = 0.95;

    //Pole
    const pole_geometry = new BoxGeometry(0.8, 5, 0.8);

    const pole = new Mesh(pole_geometry, gripper_material);
    pole.position.y += 3.7;
    pole.name = "pole";

    // Base
    const base_geometry = new BoxGeometry(3.0, 0.6, 0.8);
    const base = new Mesh(base_geometry, gripper_material);
    base.position.y += 0.9;
    base.name = "base";

    // fingers
    const finger_geometry = new BoxGeometry(0.8, 0.6, 0.8);

    const finger_right = new Mesh(finger_geometry, gripper_material);
    finger_right.position.y += 0.3;
    finger_right.position.x += 0.8;
    finger_right.name = "finger_right";

    const finger_left = new Mesh(finger_geometry, gripper_material);
    finger_left.position.y += 0.3;
    finger_left.position.x -= 0.8;
    finger_left.name = "finger_left";

    // adding to pole
    gripper.add(pole);
    gripper.add(base);
    gripper.add(finger_right);
    gripper.add(finger_left);

    // adding to scene
    scene.add(gripper);
}

export function loadYumi(scene) {
    let yumi_model = new Object3D();
    const loader = new GLTFLoader();
    const onLoad = (result, yumi) => {
        // console.log(result);
        const model = result.scene.children[0];
        model.traverse((child) => {
            if (child.isMesh && child.geometry !== undefined) {
                child.material.transparent = true;
                child.material.opacity = 0.7;
                // console.log(child.name);
            }
        });
        yumi.add(model.clone(true));
    };
    loader.load(
        yumi_path,
        gltf => onLoad(gltf, yumi_model),
    ), undefined, function (error) {
        console.log(error);
        };
    
    yumi_model.position.set(0.0, 0.0, -5.0);
    yumi_model.rotation.set(0.0, -Math.PI/2, 0.0);
    yumi_model.scale.set(20, 20, 20);

    scene.add(yumi_model);
}

export function drawArrow(pick_pos, place_pos, scene) {
    /**
     * Curve
     */
    const start = new Vector3();
    start.x = pick_pos.y * 0.8 - 8.8;
    start.y = pick_pos.z * 0.48 + 0.01;
    start.z = pick_pos.x * 0.8 - 2.4;
    
    const finish = new Vector3();
    finish.x = place_pos.y * 0.8 - 8.8;
    finish.y = place_pos.z * 0.48 + 0.01;
    finish.z = place_pos.x * 0.8 - 2.4;

    let mid = new Vector3();
    mid.add(start);
    let dist = finish.x + mid.x;
    mid.x = dist/2;
    mid.y += 8;

    const b_curve = new QuadraticBezierCurve3(
        start,
        mid,
        finish
    );
    const points = b_curve.getPoints( 100 );
    const line_geometry = new BufferGeometry().setFromPoints( points );
    const material = new LineBasicMaterial({ color: 0x0ffffb, linewidth: 2 });
    const curve = new Line( line_geometry, material );
    /**
     * Cone
     */
    const cone_geometry = new ConeGeometry( 0.2, 1, 8 );
    const cone_material = new MeshBasicMaterial({ color: 0x0ffffb });

    const cone = new Mesh( cone_geometry, cone_material );
    cone.position.set(points[98].x, points[98].y, points[98].z);

    
    const VEnd = points[100].clone();
    // fix cone's default position which points skyward.
    cone.rotateZ(Math.PI)
    cone.geometry.rotateX( Math.PI / 2 );
    cone.lookAt(VEnd);
    /**
     * Arrow
     */

    const arrow = new Group();
    arrow.add(curve);
    arrow.add(cone);
    arrow.name = 'arrow';
    scene.add(arrow);

    setTimeout(() => {
        const arrow = scene.getObjectByName('arrow');
        scene.remove(arrow);
    }, 1000);
}

export function visuError(x, y, errs, scene){
    const x_mm = y * 0.8 - 8.8;
    const y_mm = 2.5;
    const z_mm = x * 0.8 - 2.4;

    // const circle = document.createElement('div');
    // circle.id = "circle";
    // circle.innerHTML = "!";
    
    const arrowMark = document.createElement('div');
    arrowMark.id = "arrow-mark";
    
    arrowMark.innerHTML = 'Erreur <svg viewBox="0 0 384 512"><path fill="currentColor" d="M224 402.7V32c0-17.7-14.3-32-32-32s-32 14.3-32 32v370.7l-73.4-73.3c-12.5-12.5-32.8-12.5-45.3 0s-12.5 32.8 0 45.3l128 128c12.5 12.5 32.8 12.5 45.3 0l128-128c12.5-12.5 12.5-32.8 0-45.3s-32.8-12.5-45.3 0L224 402.7z"></path></svg>'
    const objectCSS = new CSS2DObject(arrowMark);
    // const objectCSS = new CSS2DObject(circle);
    objectCSS.position.set(x_mm, y_mm, z_mm);
    objectCSS.name = 'exc_' + x + y;

    scene.add(objectCSS);
    errs.push(objectCSS);
}

export function finish(scene) { 
    const fini = document.createElement('div');
    fini.id = "finish";
    fini.innerHTML = "Bon travail, Félicitations!";
    
    const objectCSS = new CSS2DObject(fini);
    objectCSS.position.set(0, 0, 0);
    objectCSS.name = 'finish_signal';
    scene.add(objectCSS);
}