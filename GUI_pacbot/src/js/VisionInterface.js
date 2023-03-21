import ROSInterface from "./ROSInterface.js";
import CreateLego from "./LegoCreator.js";
import { finish, visuError } from "./ModelCreator.js";
import { Vector3, Group } from "three";

export default class VisionInterface {
    constructor() {
        this.iface = new ROSInterface();
        this.old_model = "";
        this.old_errs = "";
        this.corners = {
            lego_101: { size: 2, position: { x: 1, y:  7, z: 0 }, rotation: false, color: "g" },
            lego_102: { size: 2, position: { x: 1, y: 15, z: 0 }, rotation: false, color: "g" },
            lego_103: { size: 2, position: { x: 7, y:  7, z: 0 }, rotation: false, color: "g" },
            lego_104: { size: 2, position: { x: 7, y: 15, z: 0 }, rotation: false, color: "g" }
        };
    }
    
    init = (renderer, scene, camera) => {
        this.renderer = renderer;
        this.scene = scene;
        this.camera = camera;
        this.createCorners();
        this.errs_list = [];
    }

    createCorners=()=> { 
        let pos;
        for (const corner of Object.keys(this.corners)) {
            pos = new Vector3(
                this.corners[corner]["position"]["x"],
                this.corners[corner]["position"]["y"],
                this.corners[corner]["position"]["z"]
            );
            new CreateLego(this.scene,
                this.corners[corner]["size"],
                this.corners[corner]["color"],
                corner,
                pos,
                this.corners[corner]["rotation"]);
        }
    }

    rosIface = (msg) => {
        if (this.old_model.localeCompare(msg) !== 0) {
            this.old_model = msg;
            this.scene.remove(this.assembly);
            this.assembly = new Group();
            this.assembly.name = "assembly";

            const msg_obj = JSON.parse(msg);
            // console.log(msg_obj);
            for (let lego in msg_obj) {

                if (msg_obj[lego]) {
                    let lego_id = lego;
                    let size = msg_obj[lego].length;
                    let p_xx_yy = msg_obj[lego][0].split("_");
                    let x = Number(p_xx_yy[1]);
                    let y = Number(p_xx_yy[2]);
                    let z = msg_obj[lego][size - 1] - 1;
                    let c = lego[0];
                    
                    let rot = false;
                    if (size === 4) {
                        z = msg_obj[lego][size - 2] - 1;
                        rot = msg_obj[lego][size - 1];
                    }
                    let lego_pos = new Vector3(x, y, z);
                    new CreateLego(this.assembly, size, c, lego_id, lego_pos, rot)
                }
            }
            this.scene.add(this.assembly);
        }
    }

    errsIface = (msg) => {
        if ((this.old_errs.localeCompare(msg) !== 0)) {
            this.old_errs = msg;

            this.errs_list.forEach((item) => {
                this.scene.remove(item);
            });
            
            this.errs_list = [];
                        
            const msg_obj = JSON.parse(msg);
            // console.log(msg_obj);
            for (let k in msg_obj) {
                let p_xx_yy = msg_obj[k].split("_");
                let x = Number(p_xx_yy[1]);
                let y = Number(p_xx_yy[2]);
                visuError(x, y, this.errs_list, this.scene);
            }
        }
    }

    finiIface = (msg) => {
        if (msg === true) {
            finish(this.scene);
        }
    }

    updateLegos = () => {
        // create a listener to retrieve world model from camera
        this.iface.subscriber('/lego_map', 'std_msgs/String', this.rosIface);
        this.iface.subscriber('/assembly_errors', 'std_msgs/String', this.errsIface);
        this.iface.subscriber('/problem_solved', 'std_msgs/Bool', this.finiIface);
    }
}