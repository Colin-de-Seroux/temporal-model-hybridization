package fr.pir.model;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;
import jakarta.persistence.ManyToOne;
import jakarta.persistence.Table;

import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
@Entity
@Table(name = "nodes")
public class Node {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(nullable = false)
    private String name;

    /**
     * The estimated time of the node in milliseconds.
     */
    @Column(nullable = false)
    private double estimatedTime;

    /**
     * Values simulated before training the model.
     */
    private double[] executionTimes;

    // TODO : Other values like subscriber, publisher, etc.
    
    @ManyToOne
	private Procedure procedure;

}
