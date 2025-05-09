package fr.pir.model;

import java.util.HashSet;
import java.util.Set;

import jakarta.persistence.CascadeType;
import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.FetchType;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;
import jakarta.persistence.OneToMany;
import jakarta.persistence.Table;

import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
@Entity
@Table(name = "procedures")
public class Procedure {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(nullable = false, unique = true)
    private String name;

    /**
     * The estimated time of the procedure in milliseconds.
     */
    @Column(nullable = false)
    private double estimatedTime;

    /**
     * The predicted time of the procedure in milliseconds.
     */
    private double predictedTime;

    @Column(nullable = false)
    @OneToMany(mappedBy = "procedure", fetch = FetchType.EAGER, cascade = CascadeType.ALL)
    private Set<Node> nodes = new HashSet<>();

}
