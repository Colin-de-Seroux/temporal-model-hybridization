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

    @Transactional
    public Model saveJsonGraph(MultipartFile file) throws IOException {
        L.function("file name : {}", file.getOriginalFilename());

        String content = new String(file.getBytes(), StandardCharsets.UTF_8);

        Gson gson = new Gson();
        Model gsonModel = gson.fromJson(content, Model.class);
        Model model = new Model(gsonModel.getName());

        this.modelRepository.save(model);

        for (Node node : gsonModel.getNodes()) {
            node.setModel(model);
            this.nodeRepository.save(node);

            int index = 0;

            for (Behavior behavior : node.getBehaviors()) {
                behavior.setNode(node);
                behavior.setBehaviorIndex(index++);
                this.behaviorRepository.save(behavior);

                int order = 0;

                for (Action action : behavior.getActions()) {
                    action.setBehavior(behavior);
                    action.setActionOrder(order++);
                    this.actionRepository.save(action);
                }
            }
        }

        gsonModel.setId(model.getId());

        return gsonModel;
    }

}
