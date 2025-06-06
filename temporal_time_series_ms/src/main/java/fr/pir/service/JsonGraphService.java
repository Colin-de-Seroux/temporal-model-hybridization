package fr.pir.service;

import java.io.IOException;
import java.nio.charset.StandardCharsets;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.springframework.web.multipart.MultipartFile;

import com.google.gson.Gson;

import fr.phenix333.logger.MyLogger;
import fr.pir.model.Action;
import fr.pir.model.Behavior;
import fr.pir.model.Model;
import fr.pir.model.Node;
import fr.pir.repository.ActionRepository;
import fr.pir.repository.BehaviorRepository;
import fr.pir.repository.ModelRepository;
import fr.pir.repository.NodeRepository;

import jakarta.transaction.Transactional;

@Service
public class JsonGraphService {

    private static final MyLogger L = MyLogger.create(JsonGraphService.class);

    @Autowired
    private ModelRepository modelRepository;

    @Autowired
    private NodeRepository nodeRepository;

    @Autowired
    private BehaviorRepository behaviorRepository;

    @Autowired
    private ActionRepository actionRepository;

    private Action createAction(Action action, Behavior behavior, int order) {
        L.function("action order : {}", order);

        action.setBehavior(behavior);
        action.setActionOrder(order);

        return this.actionRepository.save(action);
    }

    private Behavior createBehavior(Behavior behavior, Node node, int index) {
        L.function("behavior index : {}", index);

        behavior.setNode(node);
        behavior.setBehaviorIndex(index);

        return this.behaviorRepository.save(behavior);
    }

    private Node createNode(Node node, Model model) {
        L.function("node name : {}", node.getName());

        node.setModel(model);

        return this.nodeRepository.save(node);
    }

    public Model createModel(String name) {
        L.function("model name : {}", name);

        Model model = new Model(name);
        
        return this.modelRepository.save(model);
    }

    @Transactional
    public Model saveJsonGraph(MultipartFile file) throws IOException {
        L.function("file name : {}", file.getOriginalFilename());

        String content = new String(file.getBytes(), StandardCharsets.UTF_8);

        Gson gson = new Gson();
        Model gsonModel = gson.fromJson(content, Model.class);

        Model model = this.createModel(gsonModel.getName());

        for (Node node : gsonModel.getNodes()) {
            this.createNode(node, model);

            int index = 0;

            for (Behavior behavior : node.getBehaviors()) {
                this.createBehavior(behavior, node, index++);

                int order = 0;

                for (Action action : behavior.getActions()) {
                    this.createAction(action, behavior, order++);
                }
            }
        }

        gsonModel.setId(model.getId());

        return gsonModel;
    }

}
