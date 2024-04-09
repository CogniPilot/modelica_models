within Lie2;
  operator record OpGroupElement
    Real r[Rn.Group.n_param];
  
    encapsulated operator '*'
      import Lie2.Rn.Group;
      function product = Group.product;
    end '*';
  
    function inverse = Group.inverse(a = GroupElement(r = r));
    function log = Group.log(a = GroupElement(r = r));
    Real r[Rn.Group.n_param];
    Real r[Rn.Group.n_param];
    Real r[Rn.Group.n_param];
  end OpGroupElement;

package OperatorRecords
end OperatorRecords;
