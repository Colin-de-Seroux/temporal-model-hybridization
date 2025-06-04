package fr.pir.model;

import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.annotation.JsonIgnore;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;
import jakarta.persistence.ManyToOne;
import jakarta.persistence.Table;

import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@Getter
@NoArgsConstructor
@Setter
@Entity
@Table(name = "actions")
public class Action {

    @Id
    @GeneratedValue(strategy = GenerationType.AUTO)
    private Long id;

    @Column(nullable = false)
    private String type;

    @Column(nullable = false)
    private int actionOrder;

    /**
     * For pub / sub
     */
    private String topic;

    /**
     * For timer / action / service
     */
    private double value;

    /**
     *
     * Execution time for delta of sub-pub / timmer / action / service
     *
     */
    /**
     * Values simulated before training the model.
     */
    private List<Double> executionTimes = new ArrayList<>();

    /**
     * The estimated time of the node in milliseconds by the AI.
     */
    private double estimatedExecTime;

    @JsonIgnore
    @ManyToOne
    private Behavior behavior;

}
