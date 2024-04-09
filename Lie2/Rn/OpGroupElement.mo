within Lie2.Rn;

operator record OpGroupElement
  Real r[Rn.Group.n_param];

  encapsulated operator '*'
    import Lie2.Rn.Group;
    function product = Group.product;
  end '*';

  function inverse = Group.inverse(a = GroupElement(r = r));
  function log = Group.log(a = GroupElement(r = r));
end OpGroupElement;
